"""Camera-image injection: composite a fake vehicle into a shared camera frame.

The attacker broadcasts a camera image (as part of raw-data cooperative
perception) in which a vehicle that isn't there has been inserted at a chosen
world position. The inserted pixels come from a pluggable
:class:`VehicleImageGenerator`:

* :class:`ProceduralCarGenerator` -- offline, deterministic car sprite (default,
  so the pipeline is fully reproducible with no external model).
* :class:`ExternalCommandGenerator` -- shells out to *any* AI image generator
  (Stable Diffusion, ComfyUI, an API...) that writes an RGBA cutout, so you can
  drop in a photoreal vehicle without touching the attack code.

Placement is geometric: the world point is projected with a CARLA-compatible
pinhole model and the sprite is scaled by ``f * real_size / depth``, so the fake
car sits at the right pixel and the right size for its distance.
"""
from __future__ import annotations

import io
import math
import os
import subprocess
import tempfile
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .base import Attack

Vec3 = Tuple[float, float, float]


# --------------------------------------------------------------------------- #
# Pinhole projector (CARLA-compatible)                                         #
# --------------------------------------------------------------------------- #
def _carla_rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """CARLA's local->world rotation (degrees, left-handed), from its source."""
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    m = np.identity(3)
    m[0, 0] = cp * cy
    m[0, 1] = cy * sp * sr - sy * cr
    m[0, 2] = -cy * sp * cr - sy * sr
    m[1, 0] = sy * cp
    m[1, 1] = sy * sp * sr + cy * cr
    m[1, 2] = -sy * sp * cr + cy * sr
    m[2, 0] = sp
    m[2, 1] = -cp * sr
    m[2, 2] = cp * cr
    return m


class CameraProjector:
    """World -> pixel projection using intrinsics K and world->camera matrix."""

    def __init__(self, width: int, height: int, fov_deg: float,
                 world_2_camera: np.ndarray):
        self.width = int(width)
        self.height = int(height)
        self.fov_deg = float(fov_deg)
        f = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
        self.f = f
        self.K = np.array([[f, 0, width / 2.0],
                           [0, f, height / 2.0],
                           [0, 0, 1.0]])
        self.world_2_camera = np.asarray(world_2_camera, dtype=np.float64)

    @classmethod
    def from_pose(cls, location: Vec3, yaw_deg: float, width: int, height: int,
                  fov_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0
                  ) -> "CameraProjector":
        R = _carla_rotation_matrix(yaw_deg, pitch_deg, roll_deg)
        cam2world = np.identity(4)
        cam2world[:3, :3] = R
        cam2world[:3, 3] = np.asarray(location, dtype=np.float64)
        return cls(width, height, fov_deg, np.linalg.inv(cam2world))

    @classmethod
    def from_carla(cls, camera_actor) -> "CameraProjector":
        attrs = camera_actor.attributes
        width = int(attrs["image_size_x"])
        height = int(attrs["image_size_y"])
        fov = float(attrs["fov"])
        w2c = np.array(camera_actor.get_transform().get_inverse_matrix())
        return cls(width, height, fov, w2c)

    def project(self, world_point: Vec3) -> Optional[Tuple[float, float, float]]:
        """Return (u, v, depth) in pixels, or None if behind the camera."""
        p = np.array([world_point[0], world_point[1], world_point[2], 1.0])
        cam = self.world_2_camera @ p            # UE camera frame: x fwd, y right, z up
        # UE -> standard image frame: [y, -z, x]
        point = np.array([cam[1], -cam[2], cam[0]])
        if point[2] <= 0:                        # behind the camera
            return None
        uv = self.K @ point
        return (uv[0] / uv[2], uv[1] / uv[2], float(point[2]))


# --------------------------------------------------------------------------- #
# Vehicle image generators                                                     #
# --------------------------------------------------------------------------- #
class VehicleImageGenerator:
    """Interface: produce an RGBA sprite of a vehicle at the requested size."""

    def generate(self, width_px: int, height_px: int, classification: str = "car",
                 seed: int = 0) -> Image.Image:  # pragma: no cover - interface
        raise NotImplementedError


class ProceduralCarGenerator(VehicleImageGenerator):
    """Deterministic, offline car silhouette (rear view). No external model."""

    def __init__(self, color=(30, 30, 40, 255)):
        self.color = color

    def generate(self, width_px: int, height_px: int, classification: str = "car",
                 seed: int = 0) -> Image.Image:
        w = max(8, int(width_px))
        h = max(6, int(height_px))
        img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        body_top = int(h * 0.35)
        # cabin
        d.rounded_rectangle([int(w*0.18), 0, int(w*0.82), body_top+int(h*0.15)],
                            radius=max(2, w // 12), fill=self.color)
        # body
        d.rounded_rectangle([0, body_top, w - 1, h - 1],
                            radius=max(2, w // 14), fill=self.color)
        # rear window
        d.rectangle([int(w*0.26), int(h*0.10), int(w*0.74), body_top],
                    fill=(90, 110, 130, 255))
        # tail lights
        d.rectangle([int(w*0.03), int(h*0.55), int(w*0.16), int(h*0.72)],
                    fill=(200, 40, 40, 255))
        d.rectangle([int(w*0.84), int(h*0.55), int(w*0.97), int(h*0.72)],
                    fill=(200, 40, 40, 255))
        # wheels
        d.rectangle([int(w*0.10), h-1-int(h*0.14), int(w*0.28), h-1],
                    fill=(15, 15, 15, 255))
        d.rectangle([int(w*0.72), h-1-int(h*0.14), int(w*0.90), h-1],
                    fill=(15, 15, 15, 255))
        return img


class ExternalCommandGenerator(VehicleImageGenerator):
    """Delegate sprite creation to any external AI image generator.

    ``command`` is a template run via the shell; the following placeholders are
    substituted before execution:
        {out}    path the generator must write an RGBA PNG to
        {w},{h}  requested pixel size
        {cls}    classification (car/truck/...)
        {seed}   deterministic seed
    Example:
        ExternalCommandGenerator(
            "python sd_inpaint.py --prompt 'a {cls}, rear view' "
            "--w {w} --h {h} --seed {seed} --out {out}")
    The result is loaded, converted to RGBA and resized to (w, h).
    """

    def __init__(self, command: str, timeout: int = 120):
        self.command = command
        self.timeout = timeout

    def generate(self, width_px: int, height_px: int, classification: str = "car",
                 seed: int = 0) -> Image.Image:
        w = max(8, int(width_px)); h = max(6, int(height_px))
        fd, out = tempfile.mkstemp(suffix=".png"); os.close(fd)
        cmd = self.command.format(out=out, w=w, h=h, cls=classification, seed=seed)
        subprocess.run(cmd, shell=True, check=True, timeout=self.timeout)
        img = Image.open(out).convert("RGBA").resize((w, h))
        try:
            os.remove(out)
        except OSError:
            pass
        return img


# --------------------------------------------------------------------------- #
# Attack                                                                       #
# --------------------------------------------------------------------------- #
@dataclass
class CameraInjectionAttack(Attack):
    position: Vec3 = (0.0, 0.0, 0.0)         # world coords of the fake vehicle
    dimensions: Vec3 = (4.5, 2.0, 1.5)       # (length, width, height) metres
    classification: str = "car"
    generator: VehicleImageGenerator = field(default_factory=ProceduralCarGenerator)
    seed: int = 0
    name: str = field(default="camera", init=False)

    def __post_init__(self):
        Attack.__init__(self)

    def apply_image(self, image, projector: Optional[CameraProjector] = None):
        if projector is None:
            raise ValueError("CameraInjectionAttack.apply_image needs a projector")
        frame = _to_rgba(image)
        proj = projector.project(self.position)
        if proj is None:
            self.result.notes = "fake vehicle behind camera; not injected"
            return _match_input(image, frame)
        u, v, depth = proj
        # pixel size from real-world extent at this depth
        length, width_m, height_m = self.dimensions
        px_w = max(6.0, projector.f * width_m / depth)
        px_h = max(5.0, projector.f * height_m / depth)
        sprite = self.generator.generate(int(px_w), int(px_h),
                                         self.classification, self.seed)
        # place so the sprite's base sits at the object's ground contact:
        # v is the projection of the object centre; shift down by half height.
        top_left_x = int(round(u - px_w / 2.0))
        top_left_y = int(round(v - px_h / 2.0))
        if -px_w < top_left_x < projector.width and -px_h < top_left_y < projector.height:
            frame.alpha_composite(sprite, (top_left_x, top_left_y))
            self.result.image_injections = [{
                "world_pos": [round(c, 2) for c in self.position],
                "pixel": [round(u, 1), round(v, 1)],
                "size_px": [int(px_w), int(px_h)],
                "depth_m": round(depth, 2),
                "classification": self.classification,
            }]
            self.result.notes = f"injected {self.classification} at pixel ({u:.0f},{v:.0f})"
        else:
            self.result.notes = "fake vehicle projects outside frame; not injected"
        return _match_input(image, frame)


def _to_rgba(image) -> Image.Image:
    if isinstance(image, Image.Image):
        return image.convert("RGBA")
    arr = np.asarray(image)
    if arr.ndim == 2:
        return Image.fromarray(arr).convert("RGBA")
    if arr.shape[2] == 4:
        return Image.fromarray(arr, "RGBA")
    return Image.fromarray(arr[:, :, :3], "RGB").convert("RGBA")


def _match_input(original, rgba: Image.Image):
    """Return the composited frame in the same type/shape as the input."""
    if isinstance(original, Image.Image):
        return rgba
    arr = np.asarray(original)
    if arr.ndim == 3 and arr.shape[2] == 3:
        return np.asarray(rgba.convert("RGB"))
    return np.asarray(rgba)
