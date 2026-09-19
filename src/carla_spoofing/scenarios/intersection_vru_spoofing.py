"""VRU Crossing Warning spoofing: a drone impersonating an RSU walks an ego car
through a blind intersection over a pedestrian and a cyclist.

Companion scenario to ``do_not_pass_spoofing``: same attack (identity spoofing
of a Road Side Unit, see the whitepaper's Fig. 2), same causal discipline (the
ego is driven by a controller that only reacts to received messages, never by a
scripted input) -- applied to a different use case: a Vulnerable Road User
(VRU) alert at a blind corner instead of a Do-Not-Pass Warning on a straight.

The scene (a four-way, low-visibility intersection)
-----------------------------------------------------
* **ego (Rx)**        -- approaches the intersection on the main road and must
                         decide, before it reaches the stop line, whether to
                         cross or wait.
* **cyclist / pedestrian** -- real, and closing on the crossing from the side
                         street to the ego's right. A building on the corner
                         blocks the ego's own sensors from seeing them (see
                         ``BLIND_CORNER_RANGE_M``) -- exactly why it needs the
                         RSU.
* **RSU**              -- a static roadside station with a clear view of the
                         side street, broadcasting a CPM of it every round.
* **drone**            -- the attacker, hovering at the intersection.

The attack
----------
The drone broadcasts a CPM stamped with the **RSU's station id**, listing
everything the RSU would report *except* the cyclist and the pedestrian (see
``RemoveObjectsAttack``: this scene needs two victims erased by one forged
message, not one). Because a receiver keeps one entry per station, the forgery
replaces the genuine RSU report rather than adding to it (see ``fusion``), so
both hazards vanish from the ego's fused world model. Its VRU Crossing Warning
flips STOP -> GO and it drives straight through the crossing while they are
still in it.

Closed loop, not choreography
------------------------------
The ego is driven by :class:`~carla_spoofing.control.IntersectionApproachController`,
whose only trigger for braking is the crossing decision computed from received
messages. There is no timed brake input anywhere in this file. Run it with
``--run honest`` and the ego stops well short of the crossing, waits for both
of them to clear, then goes; run ``--run spoofed`` and it never even slows
down, with nothing else changed between the two runs.

Examples
--------
    # No simulator needed: kinematic mock of the same scene.
    carla-vru-warning --mode mock --run both

    # Live sim (start it with `make up` first; this loads Town01 and clears
    # stray traffic itself).
    carla-vru-warning --mode carla --host carla-sim --run both

=====================================================================
Português (tradução do docstring acima)
=====================================================================
Spoofing do Aviso de Cruzamento de VRU: um drone se passando por uma RSU
("Road Side Unit" -- unidade de infraestrutura à beira da via) faz um carro
ego atravessar um cruzamento com baixa visibilidade por cima de um pedestre
e de um ciclista.

Cenário irmão do ``do_not_pass_spoofing``: mesmo ataque (falsificação de
identidade de uma RSU, ver Fig. 2 do whitepaper), mesma disciplina causal (o
ego é conduzido por um controlador que só reage a mensagens recebidas, nunca
por uma entrada roteirizada) -- aplicado a um caso de uso diferente: um
alerta de Usuário Vulnerável da Via (VRU -- pedestre/ciclista) numa esquina
sem visibilidade, em vez de um Aviso de Não Ultrapassar numa reta.

A cena (um cruzamento de quatro vias, com baixa visibilidade)
-----------------------------------------------------
* **ego (Rx)**        -- se aproxima do cruzamento pela via principal e
                         precisa decidir, antes de chegar na linha de parada,
                         se atravessa ou espera.
* **ciclista / pedestre** -- reais, e se aproximando do cruzamento pela rua
                         transversal à direita do ego. Um prédio na esquina
                         bloqueia os próprios sensores do ego de enxergá-los
                         (ver ``BLIND_CORNER_RANGE_M``) -- exatamente por
                         isso ele precisa confiar na RSU.
* **RSU**              -- uma estação estática à beira da via com visão
                         limpa da rua transversal, transmitindo um CPM dela
                         a cada rodada.
* **drone**            -- o atacante, pairando sobre o cruzamento.

O ataque
----------
O drone transmite um CPM carimbado com o **id de estação da RSU**, listando
tudo que a RSU reportaria *exceto* o ciclista e o pedestre (ver
``RemoveObjectsAttack``: esta cena precisa de duas vítimas apagadas por uma
única mensagem forjada, não uma só). Como um receptor mantém uma entrada por
estação, a falsificação substitui o relatório genuíno da RSU em vez de se
somar a ele (ver ``fusion``), de modo que os dois perigos desaparecem do
modelo de mundo fundido do ego. O Aviso de Cruzamento de VRU dele vira de
STOP para GO e ele atravessa o cruzamento direto enquanto os dois ainda
estão nele.

Malha fechada, não coreografia
------------------------------
O ego é conduzido por
:class:`~carla_spoofing.control.IntersectionApproachController`, cujo único
gatilho de frenagem é a decisão de cruzamento calculada a partir das
mensagens recebidas. Não há nenhuma entrada de freio cronometrada em nenhum
lugar deste arquivo. Rode com ``--run honest`` e o ego para bem antes do
cruzamento, espera os dois liberarem, e então segue; rode com
``--run spoofed`` e ele nem desacelera, sem nenhuma outra mudança entre as
duas execuções.

Exemplos
--------
    # Não precisa de simulador: mock cinemático da mesma cena.
    carla-vru-warning --mode mock --run both

    # Simulação ao vivo (inicie com `make up` primeiro; isso carrega o
    # Town01 e limpa tráfego perdido sozinho).
    carla-vru-warning --mode carla --host carla-sim --run both
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..attacks import (
    ForgedIdentity, IdentitySpoofAttack, RemoveObjectsAttack, RemoveTarget,
)
from ..control import (
    KMH, CROSSING, STOPPED, CarlaLaneReference, ConstantSpeedController,
    CrossingParams, IntersectionApproachController, KinematicVehicle,
    StraightLaneReference, VehicleCommand,
)
from ..do_not_pass_warning import EgoState, heading_unit, relative_to_ego
from ..drone import place_at as place_drone_at
from ..fusion import fuse_latest_by_station
from ..perception import ObjectState, build_cpm_from_objects, carla_states_from_snapshot
from ..report import CrossingDecisionWriter, ReportWriter
from ..v2x.cpm import CollectivePerceptionMessage
from ..v2x.packet import FileSink, NullSink, Packet, UdpSink
from ..vru_warning import STOP, VRU_CLASSES, VRUCrossingDecision, evaluate_crossing
from .do_not_pass_spoofing import _ensure_map, _find_drone, _spawn_rsu_landmark

Vec3 = Tuple[float, float, float]

MAP_NAME = "Town01"
WEATHER = "CloudyNoon"

# The ego cannot see round the corner until it is almost on top of it -- that
# is the whole premise of a *blind* intersection, and why it must trust the
# RSU. Modelled as a hard range cap on the ego's own perception of VRU classes
# only (vehicles on the main road are still seen at the normal range): a corner
# building's angular shadow would work too (see do_not_pass_spoofing's use of
# ``fusion.is_occluded`` for the lead vehicle), but here the blocker has no
# actor of its own to hang a position off, so a short range cap is the simpler
# honest model of the same effect.
# PT: o ego não enxerga a esquina até estar quase em cima dela -- essa é a
# premissa de um cruzamento *cego*, e por isso ele precisa confiar na RSU.
# Modelado como um limite rígido de alcance só na percepção do ego para
# classes VRU (veículos na via principal continuam vistos no alcance normal):
# a sombra angular de um prédio na esquina também funcionaria (ver o uso de
# ``fusion.is_occluded`` em do_not_pass_spoofing para o veículo da frente),
# mas aqui o bloqueador não tem ator próprio para ancorar uma posição, então
# um limite curto de alcance é o modelo honesto mais simples do mesmo efeito.
BLIND_CORNER_RANGE_M = 8.0

RSU_STATION_ID = 9101         # infrastructure ids live above the CARLA actor range
                               # PT: ids de infraestrutura ficam acima da faixa de atores do CARLA
VIRTUAL_DRONE_STATION_ID = 9102

# Mock-backend ids (no CARLA actors involved).
# PT: ids do backend mock (sem nenhum ator do CARLA envolvido).
MOCK_EGO_ID, MOCK_CYCLIST_ID, MOCK_PEDESTRIAN_ID = 111, 112, 113

# Drone hover spot in the carla backend: this many metres BEFORE the junction
# (the crossing point where the actual attack/collision plays out), along the
# ego's approach heading, facing the same way the ego drives -- so the
# crossing itself is what ends up laid out in front of the camera, not the
# empty road behind the ego's start. The onboard camera has no pitch (see
# AirSimConfig/settings.json), only yaw is steerable (see drone.place_at), so
# getting this heading right is what puts the car in frame at all -- a drone
# hovering with the default yaw=0 faces world +X regardless of which way the
# road runs. Lower DRONE_BACK_M / DRONE_HEIGHT_M (or pass --drone-back /
# --drone-height) to sit right on top of the crossing instead.
# PT: ponto de pairagem do drone no backend carla: essa distância ANTES do
# cruzamento (o ponto de travessia onde o ataque/atropelamento de fato
# acontece), ao longo do rumo de aproximação do ego, olhando na mesma
# direção que ele dirige -- assim é o cruzamento em si que fica na frente da
# câmera, não a via vazia atrás do início do ego. A câmera de bordo não tem
# pitch (ver AirSimConfig/settings.json), só o yaw é ajustável (ver
# drone.place_at), então acertar esse rumo é o que põe o carro no quadro --
# um drone pairando com o yaw padrão (0) encara +X do mundo, não importa a
# direção da via. Diminua DRONE_BACK_M / DRONE_HEIGHT_M (ou passe
# --drone-back / --drone-height) para ficar bem em cima do cruzamento.
DRONE_BACK_M = 10.0
DRONE_HEIGHT_M = 10.0

HONEST, SPOOFED = "honest", "spoofed"


@dataclass
class ScenarioConfig:
    """Everything tunable about the scene, the messages and the manoeuvre.

    PT: Tudo o que é configurável sobre a cena, as mensagens e a manobra.
    """

    run: str = SPOOFED                  # honest | spoofed
    duration_s: float = 30.0
    tick_s: float = 0.05                # control period; also the sim fixed delta
                                         # PT: período de controle; também o passo fixo da simulação
    message_rate_hz: float = 1.0        # CPM rate (ETSI CPS allows 1-10 Hz)
                                         # PT: taxa de CPM (o padrão ETSI CPS permite 1-10 Hz)

    # Perception ranges per station. The RSU sees the whole side street; the
    # ego's own sensors do not (see BLIND_CORNER_RANGE_M).
    # PT: alcances de percepção por estação. A RSU enxerga toda a rua
    # transversal; os próprios sensores do ego não (ver BLIND_CORNER_RANGE_M).
    rsu_range_m: float = 160.0
    drone_range_m: float = 160.0
    ego_range_m: float = 75.0
    blind_corner_range_m: float = BLIND_CORNER_RANGE_M

    # Geometry, measured along the ego's approach from its start.
    # PT: geometria, medida ao longo da aproximação do ego desde seu início.
    ego_back_m: float = 70.0            # ego start distance behind the crossing point
                                         # PT: distância inicial do ego atrás do ponto de cruzamento
    ego_cruise_kmh: float = 30.0
    pedestrian_speed_kmh: float = 10.0
    cyclist_speed_kmh: float = 20.0

    # Drone hover spot (carla backend only -- see DRONE_BACK_M above).
    # PT: ponto de pairagem do drone (só no backend carla -- ver DRONE_BACK_M acima).
    drone_back_m: float = DRONE_BACK_M
    drone_height_m: float = DRONE_HEIGHT_M

    clean_vehicles: bool = True
    out_dir: str = "out/vru_warning"
    rsu_position: Vec3 = (0.0, 0.0, 0.0)     # set once the scene geometry is known
                                              # PT: definida assim que a geometria da cena é conhecida
    drone_position: Vec3 = (0.0, 0.0, 0.0)

    def message_period_ticks(self) -> int:
        return max(1, int(round((1.0 / self.message_rate_hz) / self.tick_s)))


def vru_start_offset_m(cfg: ScenarioConfig, vru_speed_mps: float,
                       conflict_m: Optional[float] = None) -> float:
    """How far up the side street a VRU must start to reach the ego's lane at
    the same instant an *unwarned* ego -- cruising the whole way, exactly what
    the spoofed run drives -- would reach the crossing.

    This is the worst case the attack is meant to demonstrate: derived from the
    scene's own speeds and distances rather than a hand-picked constant, so
    retuning ``--ego-back`` or either speed keeps the scene physically
    consistent instead of quietly detuning the collision.

    ``conflict_m`` is the actual distance, along the ego's heading from its
    start, to the point the VRUs' path crosses the ego's lane. Defaults to
    ``cfg.ego_back_m`` (the mock backend's straight scene, where the ego's
    start-to-crossing distance and the ego's start-to-anchor distance are the
    same point by construction); the carla backend passes the real one in,
    since it is not always the same as ``cfg.ego_back_m`` (see the call site).

    PT: A que distância, rua acima, um VRU (pedestre/ciclista) precisa
    começar para alcançar a faixa do ego no mesmo instante em que um ego
    *não avisado* -- em velocidade de cruzeiro o percurso inteiro, exatamente
    o que a execução spoofed faz -- chegaria ao cruzamento.

    Este é o pior caso que o ataque pretende demonstrar: derivado das
    próprias velocidades e distâncias da cena em vez de uma constante fixa
    escolhida à dedo, para que reajustar ``--ego-back`` ou qualquer uma das
    velocidades mantenha a cena fisicamente consistente em vez de
    silenciosamente descalibrar a colisão.

    ``conflict_m`` é a distância real, ao longo do rumo do ego a partir do
    início dele, até o ponto onde o caminho dos VRUs cruza a faixa do ego.
    Usa ``cfg.ego_back_m`` por padrão (a cena reta do backend mock, onde a
    distância início-ao-cruzamento e início-à-âncora do ego são o mesmo
    ponto por construção); o backend carla passa o valor real, já que nem
    sempre é igual a ``cfg.ego_back_m`` (ver o ponto de chamada).
    """
    ego_cruise_mps = cfg.ego_cruise_kmh * KMH
    ego_time_to_crossing = (cfg.ego_back_m if conflict_m is None else conflict_m) / ego_cruise_mps
    return vru_speed_mps * ego_time_to_crossing



# --------------------------------------------------------------------------- #
# Message construction and fusion (shared by both backends)                    #
# PT: Construção de mensagens e fusão (compartilhado pelos dois backends)     #
# --------------------------------------------------------------------------- #
@dataclass
class RoundMessages:
    rsu: CollectivePerceptionMessage
    drone_honest: CollectivePerceptionMessage
    forged: CollectivePerceptionMessage
    ego: CollectivePerceptionMessage
    honest_objects: List
    spoofed_objects: List


def ego_self_cpm(cfg: ScenarioConfig, states: Sequence[ObjectState], ego_id: int,
                 gen_time: float) -> CollectivePerceptionMessage:
    """What the ego's own sensors see -- with the blind corner in the way.

    Without this cap the scenario would be vacuous: an ego that can already see
    the cyclist and pedestrian itself has no reason to trust the RSU, and
    suppressing the RSU's report would change nothing.

    PT: O que os próprios sensores do ego enxergam -- com a esquina cega no
    caminho.

    Sem esse limite o cenário seria vazio: um ego que já consegue ver o
    ciclista e o pedestre por conta própria não tem motivo para confiar na
    RSU, e suprimir o relatório da RSU não mudaria nada.
    """
    ego = next(s for s in states if s.object_id == ego_id)
    cpm = build_cpm_from_objects(
        station_id=ego_id, reference_position=ego.position, objects=states,
        generation_time=gen_time, perception_range=cfg.ego_range_m,
        station_type="vehicle")

    def _dist(a: Vec3, b: Vec3) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    cpm.perceived_objects = [
        o for o in cpm.perceived_objects
        if o.classification not in VRU_CLASSES
        or _dist(o.position, ego.position) <= cfg.blind_corner_range_m
    ]
    return cpm


def build_round(cfg: ScenarioConfig, states: Sequence[ObjectState], ego_id: int,
                drone_station_id: int, gen_time: float,
                attack: IdentitySpoofAttack) -> RoundMessages:
    """Build this round's genuine and forged messages, and both fused views.

    PT: Monta as mensagens genuínas e forjadas desta rodada, e as duas
    visões fundidas (com e sem o ataque).
    """
    rsu = build_cpm_from_objects(
        station_id=RSU_STATION_ID, reference_position=cfg.rsu_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.rsu_range_m, station_type="rsu")
    drone_honest = build_cpm_from_objects(
        station_id=drone_station_id, reference_position=cfg.drone_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.drone_range_m, station_type="drone")
    forged = attack.apply_cpm(drone_honest)
    ego = ego_self_cpm(cfg, states, ego_id, gen_time)

    honest_view = fuse_latest_by_station([ego, rsu])
    # Arrival order matters: the forgery carries the RSU's station id and
    # arrives after the genuine report, so it supersedes rather than
    # supplements it.
    # PT: a ordem de chegada importa: a falsificação carrega o id de estação
    # da RSU e chega depois do relatório genuíno, então ela substitui em vez
    # de complementar.
    spoofed_view = fuse_latest_by_station([ego, rsu, forged])
    return RoundMessages(rsu, drone_honest, forged, ego,
                         honest_view.objects, spoofed_view.objects)


def make_attack(cyclist_id: int, pedestrian_id: int,
                cfg: ScenarioConfig) -> IdentitySpoofAttack:
    """The drone: erase both VRUs, then sign the message as the RSU.

    PT: O drone: apaga os dois VRUs e depois assina a mensagem como se fosse
    a RSU.
    """
    return IdentitySpoofAttack(
        ForgedIdentity(station_id=RSU_STATION_ID,
                       reference_position=cfg.rsu_position,
                       station_type="rsu"),
        inner=RemoveObjectsAttack([
            RemoveTarget(object_id=cyclist_id, carve_lidar=False),
            RemoveTarget(object_id=pedestrian_id, carve_lidar=False),
        ]))


def _ego_state(states: Sequence[ObjectState], ego_id: int,
               desired_speed: Optional[float] = None) -> EgoState:
    s = next(o for o in states if o.object_id == ego_id)
    return EgoState(station_id=ego_id, position=s.position, yaw_deg=s.yaw_deg,
                    velocity=s.velocity, desired_speed_mps=desired_speed)


def truth_decision(states: Sequence[ObjectState], ego: EgoState,
                   gen_time: float) -> VRUCrossingDecision:
    """The decision an *omniscient* receiver would make: no range limit, no
    blind corner, no messages. The yardstick the experiment is scored against
    -- crossing is unsafe when ground truth says STOP, whatever the ego was
    told. Analysis only, never transmitted.

    PT: A decisão que um receptor *onisciente* tomaria: sem limite de
    alcance, sem esquina cega, sem mensagens. É a régua com que o
    experimento é avaliado -- o cruzamento é inseguro quando a verdade
    absoluta diz STOP, independente do que o ego recebeu. Usada só para
    análise, nunca é transmitida.
    """
    omniscient = build_cpm_from_objects(
        station_id=ego.station_id, reference_position=ego.position,
        objects=states, generation_time=gen_time,
        perception_range=float("inf"), station_type="vehicle")
    return evaluate_crossing(ego, omniscient.perceived_objects)


def _boxes_overlap(a: ObjectState, b: ObjectState) -> bool:
    """Crude oriented-box overlap in a's frame -- enough to flag a mock collision.

    PT: Sobreposição grosseira de caixas orientadas no referencial de `a` --
    suficiente para sinalizar uma colisão no mock.
    """
    ego = EgoState(a.object_id, a.position, a.yaw_deg, a.velocity)
    s, d = relative_to_ego(ego, b.position)
    return (abs(s) <= 0.5 * (a.dimensions[0] + b.dimensions[0])
            and abs(d) <= 0.5 * (a.dimensions[1] + b.dimensions[1]))


# --------------------------------------------------------------------------- #
# Outcome book-keeping                                                         #
# PT: Contabilização do resultado                                             #
# --------------------------------------------------------------------------- #
@dataclass
class Outcome:
    run: str
    mode: str
    stopped_s: Optional[float] = None       # when the ego first held at the stop line
                                             # PT: quando o ego parou pela primeira vez na linha de parada
    crossed_s: Optional[float] = None       # when the ego passed the point of no return
                                             # PT: quando o ego passou o ponto sem volta
    crossing_was_unsafe: bool = False       # ground truth said STOP at that moment
                                             # PT: a verdade absoluta dizia STOP naquele momento
    truth_at_crossing: str = ""
    collisions: List[dict] = field(default_factory=list)   # one entry per distinct victim, first hit
                                                             # PT: uma entrada por vítima distinta, no primeiro impacto
    both_vrus_hit: bool = False
    n_decisions: int = 0
    n_flips: int = 0
    first_flip_s: Optional[float] = None
    ego_state_changes: List = field(default_factory=list)
    attack: dict = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "run": self.run, "mode": self.mode,
            "stopped_s": self.stopped_s,
            "ego_stopped": self.stopped_s is not None,
            "crossed_s": self.crossed_s,
            "ego_crossed": self.crossed_s is not None,
            "crossing_was_unsafe": self.crossing_was_unsafe,
            "truth_at_crossing": self.truth_at_crossing,
            "collision": self.collisions[0] if self.collisions else None,
            "collisions": self.collisions,
            "collided": bool(self.collisions),
            "both_vrus_hit": self.both_vrus_hit,
            "n_decisions": self.n_decisions, "n_flips": self.n_flips,
            "first_flip_s": self.first_flip_s,
            "ego_state_changes": self.ego_state_changes,
            "attack": self.attack, "notes": self.notes,
        }


def _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time, ego_id,
               ego_speed, ego_state_name, msgs: RoundMessages,
               acted: VRUCrossingDecision, counter: VRUCrossingDecision,
               drone_station_id: int, outcome: Outcome,
               truth: Optional[VRUCrossingDecision] = None) -> None:
    hidden = sorted(set(msgs.rsu.object_ids()) - set(msgs.forged.object_ids()))
    dec_writer.add(run=cfg.run, frame=seq, sim_time=sim_time, ego_id=ego_id,
                   ego_speed=ego_speed, ego_state=ego_state_name,
                   acted=acted, counterfactual=counter, truth=truth,
                   hidden_object_id=";".join(str(i) for i in hidden) if hidden else None)
    outcome.n_decisions += 1
    if acted.decision != counter.decision:
        outcome.n_flips += 1
        if outcome.first_flip_s is None:
            outcome.first_flip_s = round(sim_time, 2)

    # Genuine traffic is always on the air; the forgery only in the attack run.
    # PT: o tráfego genuíno está sempre no ar; a falsificação só na execução com ataque.
    for cpm, sender, stype, atk in ((msgs.rsu, RSU_STATION_ID, "rsu", False),
                                    (msgs.ego, ego_id, "vehicle", False)):
        sink.send(Packet.cpm(cpm, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=sender,
                       sender_type=stype, is_attacker=atk,
                       honest=cpm, broadcast=cpm)
    if cfg.run == SPOOFED:
        sink.send(Packet.cpm(msgs.forged, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=drone_station_id,
                       sender_type="drone", is_attacker=True,
                       honest=msgs.drone_honest, broadcast=msgs.forged)


# --------------------------------------------------------------------------- #
# Mock backend: the same closed loop, on a kinematic cross intersection        #
# PT: Backend mock: a mesma malha fechada, num cruzamento cinemático          #
# --------------------------------------------------------------------------- #
def run_mock(cfg: ScenarioConfig, sink, msg_writer, dec_writer) -> Outcome:
    outcome = Outcome(run=cfg.run, mode="mock")
    ego_yaw = 0.0
    vru_yaw = -90.0    # heading from the side street, across the ego's lane
                       # PT: rumo vindo da rua transversal, cruzando a faixa do ego

    cfg.rsu_position = (cfg.ego_back_m + 6.0, 4.0, 6.0)
    cfg.drone_position = (cfg.ego_back_m - 4.0, 6.0, 15.0)

    ego_cruise_mps = cfg.ego_cruise_kmh * KMH
    pedestrian_speed_mps = cfg.pedestrian_speed_kmh * KMH
    cyclist_speed_mps = cfg.cyclist_speed_kmh * KMH
    pedestrian_start_m = vru_start_offset_m(cfg, pedestrian_speed_mps)
    cyclist_start_m = vru_start_offset_m(cfg, cyclist_speed_mps)

    ego_v = KinematicVehicle(MOCK_EGO_ID, position=(0.0, 0.0, 0.0), yaw_deg=ego_yaw,
                             speed=ego_cruise_mps, dimensions=(4.6, 2.0, 1.5))
    cyclist_v = KinematicVehicle(MOCK_CYCLIST_ID,
                                 position=(cfg.ego_back_m, cyclist_start_m, 0.0),
                                 yaw_deg=vru_yaw, speed=cyclist_speed_mps,
                                 classification="bicycle", dimensions=(1.8, 0.6, 1.6))
    pedestrian_v = KinematicVehicle(MOCK_PEDESTRIAN_ID,
                                    position=(cfg.ego_back_m, pedestrian_start_m, 0.0),
                                    yaw_deg=vru_yaw, speed=pedestrian_speed_mps,
                                    classification="pedestrian", dimensions=(0.6, 0.6, 1.8))

    ego_ctrl = IntersectionApproachController(
        StraightLaneReference((0.0, 0.0, 0.0), ego_yaw), cfg.ego_back_m,
        CrossingParams(cruise_speed_mps=ego_cruise_mps))
    cyclist_ctrl = ConstantSpeedController(
        StraightLaneReference((cfg.ego_back_m, cyclist_start_m, 0.0), vru_yaw),
        cyclist_speed_mps)
    pedestrian_ctrl = ConstantSpeedController(
        StraightLaneReference((cfg.ego_back_m, pedestrian_start_m, 0.0), vru_yaw),
        pedestrian_speed_mps)

    attack = make_attack(MOCK_CYCLIST_ID, MOCK_PEDESTRIAN_ID, cfg)
    # Safe default before any message has been received.
    # PT: valor padrão seguro antes de qualquer mensagem ter sido recebida.
    decision = VRUCrossingDecision(STOP, "no cooperative message received yet")
    period = cfg.message_period_ticks()
    n_ticks = int(cfg.duration_s / cfg.tick_s)
    seq = 0

    for tick in range(n_ticks):
        sim_time = tick * cfg.tick_s
        states = [
            ObjectState(v.object_id, v.position,
                       velocity=(heading_unit(v.yaw_deg)[0] * v.speed,
                                heading_unit(v.yaw_deg)[1] * v.speed, 0.0),
                       yaw_deg=v.yaw_deg, dimensions=v.dimensions,
                       classification=v.classification)
            for v in (ego_v, cyclist_v, pedestrian_v)
        ]
        if tick % period == 0:
            msgs = build_round(cfg, states, MOCK_EGO_ID, VIRTUAL_DRONE_STATION_ID,
                               sim_time, attack)
            ego_st = _ego_state(states, MOCK_EGO_ID, ego_ctrl.p.cruise_speed_mps)
            d_honest = evaluate_crossing(ego_st, msgs.honest_objects)
            d_spoofed = evaluate_crossing(ego_st, msgs.spoofed_objects)
            d_truth = truth_decision(states, ego_st, sim_time)
            if cfg.run == SPOOFED:
                decision, counter = d_spoofed, d_honest
            else:
                decision, counter = d_honest, d_spoofed
            _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                       MOCK_EGO_ID, ego_v.speed, ego_ctrl.state, msgs,
                       decision, counter, VIRTUAL_DRONE_STATION_ID, outcome,
                       truth=d_truth)
            seq += 1

        ego_st = _ego_state(states, MOCK_EGO_ID, ego_ctrl.p.cruise_speed_mps)
        ego_v.step(ego_ctrl.step(ego_st, decision, cfg.tick_s, sim_time), cfg.tick_s)
        cyclist_v.step(cyclist_ctrl.step(cyclist_v.state(), cfg.tick_s), cfg.tick_s)
        pedestrian_v.step(pedestrian_ctrl.step(pedestrian_v.state(), cfg.tick_s), cfg.tick_s)

        if outcome.stopped_s is None and ego_ctrl.state == STOPPED:
            outcome.stopped_s = round(sim_time, 2)
        if outcome.crossed_s is None and ego_ctrl.state == CROSSING:
            outcome.crossed_s = round(sim_time, 2)
            t_now = truth_decision(states, ego_st, sim_time)
            outcome.truth_at_crossing = t_now.decision
            outcome.crossing_was_unsafe = t_now.decision == STOP

        already_hit = {c["with_object_id"] for c in outcome.collisions}
        ego_os = states[0]
        for other in states[1:]:
            if other.object_id not in already_hit and _boxes_overlap(ego_os, other):
                outcome.collisions.append({
                    "sim_time": round(sim_time, 2), "with_object_id": other.object_id,
                    "classification": other.classification,
                    "ego_speed_mps": round(ego_v.speed, 2),
                })
                already_hit.add(other.object_id)
        outcome.both_vrus_hit = {MOCK_CYCLIST_ID, MOCK_PEDESTRIAN_ID} <= already_hit
        if outcome.both_vrus_hit:
            break

    outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
    outcome.attack = attack.result.to_dict()
    return outcome


# --------------------------------------------------------------------------- #
# CARLA backend                                                                #
# PT: Backend do CARLA                                                        #
# --------------------------------------------------------------------------- #
def _walk_back(waypoint, distance: float, start_junction_id, step: float = 5.0):
    """Walk backward along the road graph, stopping early upon reaching a
    *different* junction or a dead end instead of failing outright like a
    single ``previous(distance)`` call would. Returns ``(waypoint, distance
    actually covered)``.

    ``start_junction_id`` is the id of the junction being backed away from --
    the first few steps can still land inside its own geometry (a junction's
    waypoints extend a little past its visual footprint), which must not be
    mistaken for having reached a *second* intersection.

    PT: Caminha para trás pelo grafo de estradas, parando mais cedo ao
    atingir um cruzamento *diferente* ou um beco sem saída, em vez de falhar
    completamente como faria uma única chamada ``previous(distance)``.
    Retorna ``(waypoint, distância realmente percorrida)``.

    ``start_junction_id`` é o id do cruzamento do qual se está se afastando
    -- os primeiros passos ainda podem cair dentro da própria geometria dele
    (os waypoints de um cruzamento se estendem um pouco além de sua área
    visual), o que não deve ser confundido com ter chegado a um *segundo*
    cruzamento.
    """
    wp = waypoint
    covered = 0.0
    remaining = distance
    while remaining > 1e-6:
        step_d = min(step, remaining)
        prev = wp.previous(step_d)
        if not prev:
            break
        candidate = prev[0]
        if candidate.is_junction and candidate.get_junction().id != start_junction_id:
            break
        wp = candidate
        covered += step_d
        remaining -= step_d
    return wp, covered


# Shortest usable approach: enough road for the ego to still be reacting to a
# STOP decision, not already braking at the very edge of the junction.
# PT: menor aproximação utilizável: via suficiente para o ego ainda estar
# reagindo a uma decisão STOP, e não já freando bem na borda do cruzamento.
MIN_APPROACH_M = 15.0


def _find_intersection(carla, world, cfg: ScenarioConfig):
    """A driving waypoint some distance before a junction, the junction's own
    waypoint, and a lane through it that runs roughly perpendicular to the
    ego's approach (the side street the VRUs cross from).

    Version-safe like ``do_not_pass_spoofing._derive_transforms``: a junction
    is found by walking the road graph rather than hard-coding a spawn-point
    index, so the formation survives a CARLA version bump. Best effort -- the
    first workable junction is used, not a hand-picked "good" one (an earlier
    version required a clean 4-way crossroads, but every junction CARLA's road
    graph reported on Town01 came back 3-armed, so that requirement is gone).

    ``cfg.ego_back_m`` is a request, not a guarantee: town blocks are often
    shorter than it, so the approach is walked back in short steps and capped
    at whatever the road actually offers before another junction or a dead
    end, down to ``MIN_APPROACH_M``. ``cfg.ego_back_m`` is shrunk to match
    once a junction is chosen, so the rest of the scenario (VRU timing, the
    braking controller) stays consistent with where the ego actually starts.

    PT: Um waypoint de condução a certa distância antes de um cruzamento, o
    próprio waypoint do cruzamento, e uma faixa que passa por ele mais ou
    menos perpendicular à aproximação do ego (a rua transversal de onde os
    VRUs atravessam).

    Seguro entre versões como ``do_not_pass_spoofing._derive_transforms``:
    um cruzamento é encontrado caminhando pelo grafo de estradas em vez de
    fixar um índice de spawn-point, para que a formação sobreviva a uma
    atualização de versão do CARLA. Melhor esforço -- é usado o primeiro
    cruzamento viável, não um "bom" escolhido à dedo (uma versão anterior
    exigia um cruzamento limpo de 4 vias, mas todo cruzamento que o grafo de
    estradas do CARLA reportou no Town01 veio com 3 braços, então essa
    exigência foi removida).

    ``cfg.ego_back_m`` é um pedido, não uma garantia: quadras da cidade
    costumam ser mais curtas do que isso, então a aproximação é percorrida
    para trás em passos curtos e limitada ao que a via realmente oferece
    antes de outro cruzamento ou um beco sem saída, até o mínimo de
    ``MIN_APPROACH_M``. ``cfg.ego_back_m`` é reduzido para bater com isso
    assim que um cruzamento é escolhido, para que o resto do cenário
    (tempo dos VRUs, o controlador de frenagem) fique consistente com onde
    o ego realmente começa.
    """
    cmap = world.get_map()
    seen_junction_ids = set()
    n_junctions = 0
    for wp in cmap.generate_waypoints(2.0):
        if not wp.is_junction:
            continue
        junction = wp.get_junction()
        if junction.id in seen_junction_ids:
            continue
        seen_junction_ids.add(junction.id)
        n_junctions += 1

        approach_wp, covered = _walk_back(wp, cfg.ego_back_m, junction.id)
        if covered < MIN_APPROACH_M:
            print(f"[vru] junction {junction.id} only has a {covered:.1f}m "
                 f"approach (< {MIN_APPROACH_M:.0f}m needed); skipping.")
            continue
        ego_forward = heading_unit(approach_wp.transform.rotation.yaw)

        best_wp, best_score = None, -1.0
        for entry_wp, _exit_wp in junction.get_waypoints(carla.LaneType.Driving):
            fwd = heading_unit(entry_wp.transform.rotation.yaw)
            # |sin(angle between them)| -- 1.0 for a perpendicular lane, 0 for
            # one running parallel to the ego (i.e. the ego's own road through
            # the junction, which is not the side street).
            # PT: |sin(ângulo entre elas)| -- 1.0 para uma faixa perpendicular,
            # 0 para uma paralela ao ego (ou seja, a própria via do ego pelo
            # cruzamento, que não é a rua transversal).
            score = abs(fwd[0] * (-ego_forward[1]) + fwd[1] * ego_forward[0])
            if score > best_score:
                best_wp, best_score = entry_wp, score
        if best_wp is None:
            continue

        if covered < cfg.ego_back_m:
            print(f"[vru] block is only {covered:.1f}m before the crossroads "
                 f"(requested --ego-back {cfg.ego_back_m:.1f}m); using {covered:.1f}m.")
            cfg.ego_back_m = covered
        return approach_wp, wp, best_wp

    raise SystemExit(
        f"No junction with a long enough approach found on {MAP_NAME} "
        f"({n_junctions} junction(s) checked); try a smaller --ego-back or "
        "a different --map.")




def _spawn_vru_actors(carla, world, cfg: ScenarioConfig, cross_wp, start_junction_id,
                      cyclist_start_m: float, pedestrian_start_m: float,
                      pedestrian_clearance_m: float):
    """Place the cyclist (a physical vehicle) on the crossing lane and the
    pedestrian (a walker) on the sidewalk alongside it, both ``start_m`` back
    from the junction along it.

    ``pedestrian_clearance_m`` must match the value ``run_carla`` used to
    retime ``pedestrian_start_m`` (see the comment there) -- it is passed in
    rather than recomputed from the pedestrian's own waypoint so the two
    never drift apart and quietly detune the collision again.

    PT: Posiciona o ciclista (um veículo físico) na faixa de cruzamento e o
    pedestre (um walker) na calçada ao lado dela, ambos ``start_m`` para trás
    a partir do cruzamento ao longo dela.

    ``pedestrian_clearance_m`` precisa bater com o valor que ``run_carla``
    usou para recronometrar ``pedestrian_start_m`` (ver o comentário lá) --
    é passado em vez de recalculado a partir do waypoint do pedestre para que
    os dois nunca se desalinhem e descalibrem a colisão de novo.
    """
    bl = world.get_blueprint_library()

    def bp(name, fallback):
        found = bl.filter(name)
        return found[0] if found else bl.filter(fallback)[0]

    cross_yaw = cross_wp.transform.rotation.yaw

    def lift(t, dz=0.3):
        return carla.Transform(
            carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + dz),
            t.rotation)

    def to_sidewalk(wp):
        """Push a driving-lane waypoint sideways, clear of the lane.

        Both VRUs were being timed to reach the SAME point on the SAME lane
        at the same instant (the worst-case moment an unwarned ego also
        arrives) -- correct for the road-crossing timing, but since the
        cyclist and the pedestrian shared that one lane, they collided with
        each other right there instead of each separately crossing in front
        of the ego. Moving the pedestrian sideways keeps the timing but puts
        them on parallel paths.

        Deliberately NOT ``get_waypoint(..., lane_type=Sidewalk)``: the actual
        modelled sidewalk mesh is where street furniture (trees, benches,
        ...) lives too, and snapping onto it can drop the pedestrian right
        against one -- a raw ``WalkerControl`` push has no path-finding to
        route around it, so it just stands there blocked. A fixed offset off
        the lane's own edge stays on the clear shoulder instead.

        PT: os dois VRUs eram cronometrados para chegar no MESMO ponto da
        MESMA faixa no mesmo instante (o pior caso em que um ego não avisado
        também chegaria) -- correto para o tempo da travessia, mas como o
        ciclista e o pedestre dividiam essa faixa única, colidiam um com o
        outro bem ali em vez de cada um cruzar separadamente na frente do
        ego. Mover o pedestre para o lado mant\u00e9m o tempo mas os coloca em
        caminhos paralelos.

        Deliberadamente NÃO ``get_waypoint(..., lane_type=Sidewalk)``: a
        malha de calçada de verdade é onde mobiliário urbano (árvores,
        bancos, ...) também fica, e grudar nela pode largar o pedestre bem
        em cima de um -- um empurrão de ``WalkerControl`` bruto não tem
        navegação para contornar, então ele simplesmente fica parado,
        bloqueado. Um deslocamento fixo a partir da borda da própria faixa
        fica no acostamento livre.
        """
        right = wp.transform.get_right_vector()
        loc = wp.transform.location
        return carla.Transform(
            carla.Location(x=loc.x + right.x * pedestrian_clearance_m,
                           y=loc.y + right.y * pedestrian_clearance_m, z=loc.z),
            wp.transform.rotation)

    # A single ``previous(distance)`` call fails outright (returns []) if the
    # side street is shorter than the requested distance -- exactly the bug
    # ``_walk_back`` was written for the ego's own approach; falling back to
    # ``cross_wp`` here silently spawned both VRUs right on top of the
    # junction instead of up the side street, so they never actually crossed
    # the ego's path at the intended moment.
    # PT: uma única chamada ``previous(distance)`` falha completamente
    # (retorna []) se a rua transversal for mais curta que a distância
    # pedida -- exatamente o bug para o qual ``_walk_back`` foi escrito na
    # aproximação do próprio ego; cair de volta para ``cross_wp`` aqui fazia
    # os dois VRUs nascerem em cima do cruzamento em vez de rua acima, então
    # eles nunca cruzavam de fato o caminho do ego no momento pretendido.
    cyc_wp, _ = _walk_back(cross_wp, cyclist_start_m, start_junction_id)
    cyclist = world.try_spawn_actor(bp("vehicle.bh.crossbike", "vehicle.diamondback.century"),
                                    lift(cyc_wp.transform))

    ped_wp, _ = _walk_back(cross_wp, pedestrian_start_m, start_junction_id)
    walker_bp = bl.filter("walker.pedestrian.*")[0]
    if walker_bp.has_attribute("is_invincible"):
        walker_bp.set_attribute("is_invincible", "false")
    pedestrian = world.try_spawn_actor(walker_bp, lift(to_sidewalk(ped_wp), dz=0.5))

    return cyclist, pedestrian, cross_yaw



def _ensure_map_and_actors(carla, world, cfg, args, outcome):
    """Clear stray traffic before this scene spawns its own actors.

    PT: Remove tráfego perdido antes desta cena instanciar seus próprios atores.
    """
    existing = [a for a in world.get_actors().filter("vehicle.*")
               if "drone" not in a.type_id.lower()]
    existing += list(world.get_actors().filter("walker.pedestrian.*"))
    existing += list(world.get_actors().filter("controller.ai.walker"))
    if existing:
        if cfg.clean_vehicles:
            print(f"[vru] removing {len(existing)} pre-existing vehicles/walkers ...")
            for a in existing:
                try:
                    if a.type_id == "controller.ai.walker":
                        a.stop()
                    a.destroy()
                except Exception:
                    pass
        else:
            outcome.notes.append(
                f"--keep-vehicles: {len(existing)} other vehicles/walkers left "
                "in the world; they may disturb the scene.")
            print("[vru] WARNING: " + outcome.notes[-1])


def run_carla(cfg: ScenarioConfig, sink, msg_writer, dec_writer, args) -> Outcome:
    try:
        import carla
    except ImportError as e:
        print("ERROR: `carla` module not importable. Run inside the CarlaAir "
              "conda env / Docker image, with the sim reachable.", file=sys.stderr)
        raise SystemExit(2) from e

    outcome = Outcome(run=cfg.run, mode="carla")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world, previous = _ensure_map(client, MAP_NAME, args.timeout)
    if previous is not None:
        outcome.notes.append(f"loaded {MAP_NAME} (sim was on {previous})")
    world.set_weather(getattr(carla.WeatherParameters, WEATHER))

    _ensure_map_and_actors(carla, world, cfg, args, outcome)

    ego_cruise_mps = cfg.ego_cruise_kmh * KMH
    pedestrian_speed_mps = cfg.pedestrian_speed_kmh * KMH
    cyclist_speed_mps = cfg.cyclist_speed_kmh * KMH

    original_settings = world.get_settings()
    spawned, collision_sensor = [], None
    collisions: List[dict] = []
    ego_ctrl = attack = None
    try:
        approach_wp, junction_wp, cross_wp = _find_intersection(carla, world, cfg)
        # _find_intersection may have shrunk cfg.ego_back_m to whatever the
        # block actually offers, so the VRU timing is derived from it only now.
        # PT: _find_intersection pode ter reduzido cfg.ego_back_m ao que a
        # quadra realmente oferece, então o tempo dos VRUs só é derivado dele agora.
        ego_forward = heading_unit(approach_wp.transform.rotation.yaw)

        # The VRUs' spawn timing must be pinned to the ACTUAL point their lane
        # crosses the ego's (cross_wp), not cfg.ego_back_m/junction_wp: that is
        # just *some* waypoint CARLA's road graph marked as belonging to the
        # junction (see _find_intersection), which can sit metres away from
        # the real crossing. This only fixes where/when the VRUs start
        # walking -- it deliberately does NOT touch the ego's own
        # IntersectionApproachController below (still built with
        # cfg.ego_back_m), so the honest run's braking/stop-line behaviour is
        # unchanged. In the spoofed run the ego never brakes at all (that is
        # the whole point of the attack), so it simply cruises the entire way
        # at ego_cruise_mps -- meaning the time it *actually* takes to
        # physically reach cross_wp is conflict_m / ego_cruise_mps regardless
        # of any parameter fed to the controller, and the VRUs must be timed
        # against that real number, not the controller's stop-line one.
        # PT: a cronometragem de partida dos VRUs precisa ser presa ao ponto
        # REAL onde a faixa deles cruza a do ego (cross_wp), não a
        # cfg.ego_back_m/junction_wp: isso é só *algum* waypoint que o grafo
        # de estradas do CARLA marcou como pertencente ao cruzamento (ver
        # _find_intersection), que pode ficar metros longe do cruzamento de
        # verdade. Isto só conserta onde/quando os VRUs começam a andar --
        # deliberadamente NÃO mexe no IntersectionApproachController do ego
        # logo abaixo (continua construído com cfg.ego_back_m), então o
        # comportamento de frenagem/linha de parada da execução honest não
        # muda. Na execução spoofed o ego nunca freia (esse é o objetivo do
        # ataque), então ele simplesmente cruza o percurso inteiro em
        # ego_cruise_mps -- ou seja, o tempo que ele REALMENTE leva para
        # alcançar fisicamente cross_wp é conflict_m / ego_cruise_mps,
        # independente de qualquer parâmetro passado ao controlador, e os
        # VRUs precisam ser cronometrados contra esse número real, não o da
        # linha de parada do controlador.
        ego_start = approach_wp.transform.location
        to_cross_x = cross_wp.transform.location.x - ego_start.x
        to_cross_y = cross_wp.transform.location.y - ego_start.y
        conflict_m = to_cross_x * ego_forward[0] + to_cross_y * ego_forward[1]

        cyclist_start_m = vru_start_offset_m(cfg, cyclist_speed_mps, conflict_m)
        pedestrian_start_m = vru_start_offset_m(cfg, pedestrian_speed_mps, conflict_m)

        # The pedestrian is pushed sideways off the cyclist's lane (see
        # _spawn_vru_actors.to_sidewalk) so the two don't run into each other.
        # That push is along the side street's own right vector, which --
        # since the side street runs roughly perpendicular to the ego's road
        # -- lands almost entirely ALONG the ego's direction of travel. Left
        # uncorrected, the pedestrian would then reach the ego's lane at a
        # different point (and so a different moment) than the "unwarned ego
        # reaches the crossing" instant the whole scene is timed around, and
        # could clear the crossing before or after the ego instead of into
        # it. Retime the start distance so it still arrives exactly when the
        # ego reaches that (shifted) point.
        # PT: o pedestre é empurrado para o lado, para fora da faixa do
        # ciclista (ver _spawn_vru_actors.to_sidewalk), para que os dois não
        # se atropelem. Esse empurrão é ao longo do vetor "direita" da rua
        # transversal, que -- como essa rua é praticamente perpendicular à
        # via do ego -- cai quase inteiramente NA direção de deslocamento do
        # ego. Sem corrigir, o pedestre chegaria à faixa do ego em um ponto
        # (e portanto um instante) diferente do momento "ego não avisado
        # chega ao cruzamento" em torno do qual toda a cena é cronometrada, e
        # poderia liberar o cruzamento antes ou depois do ego em vez de na
        # frente dele. Recronometra a distância de partida para que ainda
        # chegue exatamente quando o ego alcançar esse ponto (deslocado).
        pedestrian_clearance_m = cross_wp.lane_width / 2.0 + 1.0
        cross_right = cross_wp.transform.get_right_vector()
        offset_along_ego = pedestrian_clearance_m * (
            cross_right.x * ego_forward[0] + cross_right.y * ego_forward[1])
        pedestrian_start_m += pedestrian_speed_mps * offset_along_ego / ego_cruise_mps

        anchor = junction_wp.transform.location
        cfg.rsu_position = (anchor.x, anchor.y, 6.0)

        # drone_back_m before the junction (the actual crossing action), along
        # the ego's approach heading -- not behind the ego's own start, which
        # is already far from the action, and not directly over the junction
        # with a default yaw either, which pointed the (pitch-less) camera at
        # world +X regardless of the road's direction.
        # PT: drone_back_m antes do cruzamento (onde a ação de fato acontece),
        # ao longo do rumo de aproximação do ego -- não atrás do próprio
        # início do ego, que já fica longe da ação, nem direto sobre o
        # cruzamento com um yaw padrão, que apontava a câmera (sem pitch)
        # para +X do mundo, sem relação com a direção da via.
        cfg.drone_position = (anchor.x - ego_forward[0] * cfg.drone_back_m,
                              anchor.y - ego_forward[1] * cfg.drone_back_m,
                              cfg.drone_height_m)
        drone_yaw = approach_wp.transform.rotation.yaw

        if not args.no_fly_drone:
            status = place_drone_at(world, args.host, cfg.drone_position,
                                    yaw_deg=drone_yaw, timeout_s=args.drone_timeout)
            print(f"[vru] {status}")
            outcome.notes.append(
                f"drone hover {cfg.drone_back_m:.0f} m before the junction, "
                f"{cfg.drone_height_m:.0f} m up, facing {drone_yaw:.1f} deg")

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = cfg.tick_s
        world.apply_settings(settings)

        rsu_actors, rsu_note = _spawn_rsu_landmark(carla, world, cfg)
        spawned.extend(rsu_actors)
        outcome.notes.append(rsu_note)
        print(f"[vru] {rsu_note}")

        bl = world.get_blueprint_library()
        def bp(name, fallback="vehicle.*"):
            found = bl.filter(name)
            return found[0] if found else bl.filter(fallback)[0]

        ego_tf = carla.Transform(
            carla.Location(x=approach_wp.transform.location.x,
                           y=approach_wp.transform.location.y,
                           z=approach_wp.transform.location.z + 0.3),
            approach_wp.transform.rotation)
        ego = world.try_spawn_actor(bp("vehicle.tesla.cybertruck"), ego_tf)
        cyclist, pedestrian, cross_yaw = _spawn_vru_actors(
            carla, world, cfg, cross_wp, junction_wp.get_junction().id,
            cyclist_start_m, pedestrian_start_m, pedestrian_clearance_m)
        if not all((ego, cyclist, pedestrian)):
            for a in (ego, cyclist, pedestrian):
                if a:
                    a.destroy()
            raise SystemExit("Could not spawn ego/cyclist/pedestrian (blocked "
                             "spawn points?). Try --clean-vehicles.")
        spawned += [ego, cyclist, pedestrian]
        print(f"[vru] ego={ego.id} cyclist={cyclist.id} pedestrian={pedestrian.id}")

        drone = _find_drone(world)
        drone_station_id = drone.id if drone is not None else VIRTUAL_DRONE_STATION_ID
        if drone is None:
            outcome.notes.append(
                "no drone actor in the world; the attacker is a virtual "
                "station at the intersection")
            print("[vru] " + outcome.notes[-1])

        cs_bp = bl.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(cs_bp, carla.Transform(), attach_to=ego)
        collision_sensor.listen(lambda e: collisions.append({
            "with_actor_id": e.other_actor.id, "with_type_id": e.other_actor.type_id}))

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=junction_wp.transform.location.x - 15.0,
                           y=junction_wp.transform.location.y,
                           z=junction_wp.transform.location.z + 25.0),
            carla.Rotation(pitch=-55.0)))

        for _ in range(2):
            world.tick()

        # A raw WalkerControl velocity kick, not the AI walker controller: the
        # AI controller navigates the nav-mesh toward a goal and will route
        # round the very conflict this scene needs, quietly rescuing the
        # victim exactly like CARLA's vehicle traffic manager would (see
        # do_not_pass_spoofing's note on ConstantSpeedController).
        # PT: um impulso de velocidade via WalkerControl bruto, não o
        # controlador de walker por IA: o controlador de IA navega pela
        # nav-mesh em direção a um objetivo e vai contornar exatamente o
        # conflito que esta cena precisa, salvando silenciosamente a vítima
        # tal como o traffic manager de veículos do CARLA faria (ver a nota
        # sobre ConstantSpeedController em do_not_pass_spoofing).
        ped_fwd = heading_unit(cross_yaw)
        pedestrian.apply_control(carla.WalkerControl(
            direction=carla.Vector3D(ped_fwd[0], ped_fwd[1], 0.0),
            speed=pedestrian_speed_mps))

        lane = CarlaLaneReference(world.get_map())
        ego_ctrl = IntersectionApproachController(
            lane, cfg.ego_back_m, CrossingParams(cruise_speed_mps=ego_cruise_mps))
        # NOT CarlaLaneReference: that snaps to whichever driving lane is
        # nearest, which is the ego's own road the moment the cyclist reaches
        # it -- the steering then swings to follow the main road's heading
        # instead of continuing straight across it, and the sudden turn
        # tips the bike over mid-crossing. A straight line anchored on the
        # cyclist's own spawn pose has no such lane to snap onto.
        # PT: não usar CarlaLaneReference: ela gruda na faixa de rolamento
        # mais próxima, que passa a ser a própria via do ego assim que o
        # ciclista chega nela -- a direção então vira para seguir o rumo da
        # via principal em vez de continuar reto atravessando, e a guinada
        # brusca derruba a bike no meio da travessia. Uma linha reta ancorada
        # na própria pose de nascimento do ciclista não tem essa faixa para
        # grudar.
        cyc_tf = cyclist.get_transform()
        cyclist_lane = StraightLaneReference(
            (cyc_tf.location.x, cyc_tf.location.y, cyc_tf.location.z), cross_yaw)
        cyclist_ctrl = ConstantSpeedController(cyclist_lane, cyclist_speed_mps)
        attack = make_attack(cyclist.id, pedestrian.id, cfg)

        for actor, speed in ((ego, ego_cruise_mps), (cyclist, cyclist_speed_mps)):
            fwd = actor.get_transform().get_forward_vector()
            actor.set_target_velocity(carla.Vector3D(fwd.x * speed, fwd.y * speed, 0.0))
        world.tick()

        decision = VRUCrossingDecision(STOP, "no cooperative message received yet")
        period = cfg.message_period_ticks()
        n_ticks = int(cfg.duration_s / cfg.tick_s)
        seq = 0
        wall_deadline = time.monotonic() + args.wall_timeout
        clock_origin = None
        stop_at: Optional[float] = None
        hit_ids: set = set()
        n_collisions_seen = 0
        first_hit_s: Optional[float] = None
        target_ids = {cyclist.id, pedestrian.id}

        for tick in range(n_ticks):
            if time.monotonic() > wall_deadline:
                outcome.notes.append(
                    f"wall-clock budget of {args.wall_timeout:.0f}s exhausted "
                    f"after {tick} of {n_ticks} ticks")
                print("[vru] " + outcome.notes[-1])
                break
            snap = world.get_snapshot()
            if clock_origin is None:
                clock_origin = snap.timestamp.elapsed_seconds
            sim_time = snap.timestamp.elapsed_seconds - clock_origin
            states = carla_states_from_snapshot(world, snap)
            if not any(s.object_id == ego.id for s in states):
                outcome.notes.append("ego actor vanished from the snapshot")
                break

            if tick % period == 0:
                msgs = build_round(cfg, states, ego.id, drone_station_id,
                                   sim_time, attack)
                ego_st = _ego_state(states, ego.id, ego_ctrl.p.cruise_speed_mps)
                d_honest = evaluate_crossing(ego_st, msgs.honest_objects)
                d_spoofed = evaluate_crossing(ego_st, msgs.spoofed_objects)
                d_truth = truth_decision(states, ego_st, sim_time)
                if cfg.run == SPOOFED:
                    decision, counter = d_spoofed, d_honest
                else:
                    decision, counter = d_honest, d_spoofed
                _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                           ego.id, ego_st.speed, ego_ctrl.state, msgs,
                           decision, counter, drone_station_id, outcome,
                           truth=d_truth)
                seq += 1

            ego_st = _ego_state(states, ego.id, ego_ctrl.p.cruise_speed_mps)
            # Keep driving through a first hit -- braking immediately would
            # rescue whichever VRU it hasn't reached yet, when both were timed
            # to be in its path at once (see vru_start_offset_m). Only holds
            # once both are down, or a hit has gone unmatched for crash-hold.
            # PT: continua dirigindo mesmo depois do primeiro impacto --
            # frear imediatamente salvaria o VRU que ainda não foi atingido,
            # quando os dois foram cronometrados para estar no caminho ao
            # mesmo tempo (ver vru_start_offset_m). Só segura de vez quando
            # os dois caíram, ou um impacto ficou sem par por crash-hold.
            braking = outcome.both_vrus_hit or (
                first_hit_s is not None and sim_time - first_hit_s >= args.crash_hold)
            if braking:
                brake_cmd = VehicleCommand(brake=1.0).to_carla()
                for actor in (ego, cyclist):
                    actor.apply_control(brake_cmd)
                pedestrian.apply_control(carla.WalkerControl(speed=0.0))
            else:
                ego.apply_control(ego_ctrl.step(ego_st, decision, cfg.tick_s,
                                                sim_time).to_carla())
                st = next((s for s in states if s.object_id == cyclist.id), None)
                if st is not None:
                    cyclist.apply_control(cyclist_ctrl.step(
                        EgoState(cyclist.id, st.position, st.yaw_deg, st.velocity),
                        cfg.tick_s).to_carla())
                    # vehicle.bh.crossbike/diamondback.century can't reach
                    # cyclist_speed_mps on throttle alone (weak default
                    # torque curve) -- steer via the controller above, but
                    # pin the speed every tick so cyclist_start_m's timing
                    # assumption actually holds and the crossing lines up.
                    # PT: vehicle.bh.crossbike/diamondback.century não
                    # alcança cyclist_speed_mps só no acelerador (curva de
                    # torque fraca por padrão) -- direção pelo controlador
                    # acima, mas a velocidade é fixada a cada tick para que a
                    # suposição de tempo de cyclist_start_m realmente valha e
                    # o cruzamento aconteça no momento certo.
                    cyc_fwd = cyclist.get_transform().get_forward_vector()
                    cyclist.set_target_velocity(carla.Vector3D(
                        cyc_fwd.x * cyclist_speed_mps,
                        cyc_fwd.y * cyclist_speed_mps, 0.0))

            if outcome.stopped_s is None and ego_ctrl.state == STOPPED:
                outcome.stopped_s = round(sim_time, 2)
            if outcome.crossed_s is None and ego_ctrl.state == CROSSING:
                outcome.crossed_s = round(sim_time, 2)
                t_now = truth_decision(states, ego_st, sim_time)
                outcome.truth_at_crossing = t_now.decision
                outcome.crossing_was_unsafe = t_now.decision == STOP

            world.tick()

            for ev in collisions[n_collisions_seen:]:
                actor_id = ev["with_actor_id"]
                if actor_id in hit_ids:
                    continue
                hit_ids.add(actor_id)
                outcome.collisions.append({
                    "sim_time": round(sim_time, 2),
                    "with_object_id": actor_id,
                    "classification": ev["with_type_id"],
                    "ego_speed_mps": round(ego_st.speed, 2),
                })
                print(f"[vru] collision at {sim_time:.2f}s with "
                      f"{ev['with_type_id']} ({actor_id})")
                if first_hit_s is None:
                    first_hit_s = sim_time
            n_collisions_seen = len(collisions)
            outcome.both_vrus_hit = target_ids <= hit_ids

            if braking and stop_at is None:
                stop_at = sim_time + args.crash_hold
            if stop_at is not None and sim_time >= stop_at:
                break

        if args.linger > 0:
            print(f"[vru] holding the finished scene for {args.linger:.0f}s "
                  f"before clearing the vehicles (--linger 0 to skip) ...")
            hold_until = time.monotonic() + args.linger
            while time.monotonic() < hold_until:
                try:
                    world.tick()
                except RuntimeError:
                    break
                time.sleep(cfg.tick_s)

    except KeyboardInterrupt:
        outcome.notes.append("interrupted by the user")
        print("\n[vru] interrupted — cleaning up ...")
    except RuntimeError as exc:
        outcome.notes.append(f"simulator stopped responding: {exc}")
        print("[vru] " + outcome.notes[-1])
    finally:
        if ego_ctrl is not None:
            outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
        if attack is not None:
            outcome.attack = attack.result.to_dict()
        try:
            client.set_timeout(3.0)
        except Exception:
            pass
        if collision_sensor is not None:
            try:
                collision_sensor.stop()
                collision_sensor.destroy()
            except Exception:
                pass
        for a in spawned:
            try:
                a.destroy()
            except Exception:
                pass
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
    return outcome


# --------------------------------------------------------------------------- #
# CLI                                                                          #
# PT: Interface de linha de comando                                           #
# --------------------------------------------------------------------------- #
def _make_sink(args, out_dir):
    if args.sink == "file":
        return FileSink(os.path.join(out_dir, "v2x_packets.jsonl"))
    if args.sink == "udp":
        return UdpSink(args.udp_host, args.udp_port)
    return NullSink()


def _one_run(args, run_name: str) -> dict:
    cfg = ScenarioConfig(
        run=run_name, duration_s=args.duration, tick_s=args.tick,
        message_rate_hz=args.rate, ego_back_m=args.ego_back,
        ego_cruise_kmh=args.ego_speed,
        pedestrian_speed_kmh=args.pedestrian_speed,
        cyclist_speed_kmh=args.cyclist_speed,
        drone_back_m=args.drone_back,
        drone_height_m=args.drone_height,
        clean_vehicles=not args.keep_vehicles,
        out_dir=os.path.join(args.out, run_name))
    os.makedirs(cfg.out_dir, exist_ok=True)
    sink = _make_sink(args, cfg.out_dir)
    msg_writer = ReportWriter(cfg.out_dir)
    dec_writer = CrossingDecisionWriter(cfg.out_dir)
    try:
        if args.mode == "mock":
            outcome = run_mock(cfg, sink, msg_writer, dec_writer)
        else:
            outcome = run_carla(cfg, sink, msg_writer, dec_writer, args)
    finally:
        sink.close()
        msg_writer.close()
        dec_writer.close()

    d = outcome.to_dict()
    d["out_dir"] = cfg.out_dir
    d["messages"] = msg_writer.n_messages
    with open(os.path.join(cfg.out_dir, "run_summary.json"), "w") as fh:
        json.dump(d, fh, indent=2)
    return d


def _verdict(results: Dict[str, dict]) -> dict:
    """The headline comparison: did the attack, and only the attack, cause it?

    PT: A comparação principal: foi o ataque, e somente o ataque, que causou isso?
    """
    h, s = results.get(HONEST), results.get(SPOOFED)
    if not (h and s):
        return {}
    return {
        "honest_stopped": h["ego_stopped"],
        "spoofed_stopped": s["ego_stopped"],
        "honest_unsafe_crossing": h["crossing_was_unsafe"],
        "spoofed_unsafe_crossing": s["crossing_was_unsafe"],
        "honest_collision": h["collided"],
        "spoofed_collision": s["collided"],
        "spoofed_both_vrus_hit": s["both_vrus_hit"],
        "attack_caused_unsafe_crossing": (s["crossing_was_unsafe"]
                                          and not h["crossing_was_unsafe"]),
        "attack_caused_collision": s["collided"] and not h["collided"],
    }


def main(argv=None):
    p = argparse.ArgumentParser(
        description="VRU Crossing Warning spoofing: drone impersonates an RSU "
                    "to suppress a pedestrian and a cyclist at a blind "
                    "intersection and trigger an unsafe crossing.")
    p.add_argument("--mode", choices=["mock", "carla"], default="mock")
    p.add_argument("--run", choices=[HONEST, SPOOFED, "both"], default="both",
                   help="honest baseline, the attack, or both for the comparison")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--crash-hold", type=float, default=3.0)
    p.add_argument("--linger", type=float, default=7.0)
    p.add_argument("--wall-timeout", type=float, default=600.0)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--tick", type=float, default=0.05)
    p.add_argument("--rate", type=float, default=1.0, help="CPM rate in Hz")
    p.add_argument("--ego-back", type=float, default=45.0,
                   help="ego start distance behind the crossing point, metres")
    p.add_argument("--ego-speed", type=float, default=30.0, help="km/h")
    p.add_argument("--pedestrian-speed", type=float, default=4.5, help="km/h")
    p.add_argument("--cyclist-speed", type=float, default=14.0, help="km/h")
    p.add_argument("--drone-back", type=float, default=DRONE_BACK_M,
                   help="drone hover spot, metres BEFORE the junction/crossing, "
                        "along the ego's approach (carla mode only)")
    p.add_argument("--drone-height", type=float, default=DRONE_HEIGHT_M,
                   help="drone hover altitude, metres (carla mode only)")
    p.add_argument("--no-fly-drone", action="store_true")
    p.add_argument("--drone-timeout", type=float, default=30.0)
    p.add_argument("--keep-vehicles", action="store_true")
    p.add_argument("--sink", choices=["file", "udp", "null"], default="file")
    p.add_argument("--out", default="out/vru_warning")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=47001)
    args = p.parse_args(argv)

    runs = [HONEST, SPOOFED] if args.run == "both" else [args.run]
    results = {r: _one_run(args, r) for r in runs}
    summary = {"mode": args.mode, "map": MAP_NAME if args.mode == "carla" else "mock",
               "runs": results, "verdict": _verdict(results)}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "comparison.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))

    for name, r in results.items():
        print(f"\n[{name}] stopped={r['ego_stopped']} "
              f"collision={r['collided']} both_vrus_hit={r['both_vrus_hit']} "
              f"flips={r['n_flips']}/{r['n_decisions']} -> {r['out_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
