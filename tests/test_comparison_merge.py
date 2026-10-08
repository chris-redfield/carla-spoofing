"""comparison.json must not lose a run that an earlier invocation produced.

out/vru_warning/comparison.json held nothing but the spoofed run and an empty
verdict: a later --run spoofed overwrote the combined file and the honest run's
numbers survived only in its own run_summary.json, where nobody looks. The paper
cites comparison.json as the record of each scenario's causality verdict, so a
file that silently drops half the experiment is a reproducibility defect.
"""
import json
import os

from carla_spoofing.report import write_comparison


def _read(d):
    with open(os.path.join(d, "comparison.json")) as fh:
        return json.load(fh)


def test_a_single_run_invocation_keeps_the_earlier_run(tmp_path):
    d = str(tmp_path)
    write_comparison(d, {"runs": {"honest": {"stopped_s": 6.95, "crossed_s": 19.95}},
                         "verdict": {}}, ["honest"])
    write_comparison(d, {"runs": {"spoofed": {"crossed_s": 4.6}},
                         "verdict": {}}, ["spoofed"])

    got = _read(d)
    assert sorted(got["runs"]) == ["honest", "spoofed"]
    assert got["runs"]["honest"]["stopped_s"] == 6.95      # not lost
    assert got["runs"]["spoofed"]["crossed_s"] == 4.6
    assert got["runs_this_invocation"] == ["spoofed"]
    assert got["carried_over_runs"] == ["honest"]
    assert got["runs"]["honest"]["carried_over_from"]      # provenance recorded


def test_a_carried_over_run_does_not_earn_a_verdict(tmp_path):
    """Two runs from two invocations may not share a world state.

    Merging the data is right; recomputing a verdict across it silently is
    exactly the unearned claim the evidence guard exists to refuse.
    """
    d = str(tmp_path)
    write_comparison(d, {"runs": {"honest": {}}, "verdict": {}}, ["honest"])
    write_comparison(d, {"runs": {"spoofed": {}}, "verdict": {},
                         "evidence_valid": None}, ["spoofed"])

    got = _read(d)
    assert got["verdict"] == {}
    assert got["evidence_valid"] is None


def test_run_both_replaces_both_runs_and_carries_nothing(tmp_path):
    d = str(tmp_path)
    write_comparison(d, {"runs": {"honest": {"v": 1}, "spoofed": {"v": 1}},
                         "verdict": {"a": True}}, ["honest", "spoofed"])
    write_comparison(d, {"runs": {"honest": {"v": 2}, "spoofed": {"v": 2}},
                         "verdict": {"a": False}}, ["honest", "spoofed"])

    got = _read(d)
    assert got["runs"]["honest"]["v"] == 2        # the fresh pair wins outright
    assert "carried_over_runs" not in got
    assert "carried_over_from" not in got["runs"]["honest"]
    assert got["verdict"] == {"a": False}


def test_a_damaged_previous_file_is_not_fatal(tmp_path):
    d = str(tmp_path)
    with open(os.path.join(d, "comparison.json"), "w") as fh:
        fh.write("{ this is not json")
    write_comparison(d, {"runs": {"spoofed": {}}, "verdict": {}}, ["spoofed"])
    assert sorted(_read(d)["runs"]) == ["spoofed"]
