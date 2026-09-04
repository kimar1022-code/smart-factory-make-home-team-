#!/usr/bin/env python3
"""house_geometry 검증. 실행: python3 tools/test_house_geometry.py"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from house_geometry import (COLUMN_CENTERS, SLOTS, GripMeasure, Pose2D, alignment_ok,  # noqa: E402
                            fit_base_pose, target_tcp, wall_line_in_R, wrap_deg)


def close(a, b, tol):
    return abs(a - b) <= tol


def test_fit_recovers_rotated_base():
    truth = Pose2D(170.3, -420.7, 4.2)      # 메모리: 밑판 4.2° 회전 사례
    meas = [truth.apply(c) for c in COLUMN_CENTERS]
    pose, rms = fit_base_pose(meas)
    assert close(pose.x, truth.x, 1e-6) and close(pose.y, truth.y, 1e-6)
    assert close(pose.yaw_deg, 4.2, 1e-6) and rms < 1e-6


def test_fit_with_noise_is_within_tolerance():
    random.seed(1)
    truth = Pose2D(100.0, -300.0, -3.0)
    worst = 0.0
    for _ in range(500):
        meas = [(x + random.gauss(0, 0.15), y + random.gauss(0, 0.15))
                for x, y in (truth.apply(c) for c in COLUMN_CENTERS)]
        pose, _ = fit_base_pose(meas)
        worst = max(worst, abs(wrap_deg(pose.yaw_deg - truth.yaw_deg)))
    # 점당 0.15 mm 잡음(1σ)에서 200x130 기선이면 yaw 오차는 긴벽 허용 0.29° 의 절반 아래여야 한다
    assert worst < 0.145, worst


def test_target_tcp_places_wall_on_slot():
    T_RB = Pose2D(170.0, -420.0, 4.2)
    grip = GripMeasure(center=(40.0, -3.0), angle_deg=90.0)   # TCP 에서 40 mm 앞, 3 mm 옆, 그리퍼 +y 방향으로 놓인 벽
    for slot in SLOTS.values():
        tcp, dj6 = target_tcp(T_RB, slot, grip, current_rz_deg=180.0)
        center_R, ang_R = wall_line_in_R(tcp, grip)
        want_c = T_RB.apply(slot.center)
        want_a = wrap_deg(T_RB.yaw_deg + slot.angle_deg)
        assert close(center_R[0], want_c[0], 1e-9) and close(center_R[1], want_c[1], 1e-9), slot
        assert close(wrap_deg(ang_R - want_a), 0.0, 1e-9), slot
        assert abs(dj6) <= 180.0


def test_rotation_about_tcp_shifts_wall_center():
    # 오프셋 40 mm 인 벽을 4° 돌리면 벽 중심이 약 2.8 mm 이동해야 하고, 목표 TCP 가 그만큼 되돌려 놓아야 한다
    # 슬롯 중심의 이동(베이스 회전)과 분리하기 위해, 같은 베이스 자세에서 "오프셋 40" 과 "오프셋 0" 의 TCP 차를 본다.
    g40 = GripMeasure(center=(40.0, 0.0), angle_deg=0.0)
    g0 = GripMeasure(center=(0.0, 0.0), angle_deg=0.0)
    slot = SLOTS["LONG_Y0"]

    def offset_term(yaw):
        a, _ = target_tcp(Pose2D(0, 0, yaw), slot, g40, 0.0)
        b, _ = target_tcp(Pose2D(0, 0, yaw), slot, g0, 0.0)
        return (a.x - b.x, a.y - b.y)          # = -R(yaw)·(40, 0)

    d0, d4 = offset_term(0.0), offset_term(4.0)
    shift = math.hypot(d4[0] - d0[0], d4[1] - d0[1])
    expect = 2 * 40.0 * math.sin(math.radians(2.0))  # 반지름 40, 각 4° 의 현 길이
    assert close(shift, expect, 1e-9), (shift, expect)
    assert close(shift, 2.79, 0.01)


def test_j6_increment_is_minimal_angle():
    grip = GripMeasure(center=(0.0, 0.0), angle_deg=0.0)
    slot = SLOTS["LONG_Y0"]
    _, dj6 = target_tcp(Pose2D(0, 0, 179.0), slot, grip, current_rz_deg=-179.0)
    assert close(dj6, -2.0, 1e-9), dj6            # 358° 가 아니라 -2°
    _, dj6 = target_tcp(Pose2D(0, 0, -3.5), slot, grip, current_rz_deg=2.0)
    assert close(dj6, -5.5, 1e-9), dj6


def test_alignment_gate():
    s = SLOTS["LONG_Y0"]
    assert alignment_ok(0.2, -0.25, 0.1, s)
    assert not alignment_ok(0.4, 0.0, 0.0, s)          # 0.3 한계 초과
    assert not alignment_ok(0.0, 0.0, 0.2, s)          # 0.29*0.6=0.17 초과
    s2 = SLOTS["SHORT_X0"]
    assert alignment_ok(0.2, 0.2, 0.2, s2)
    assert not alignment_ok(0.3, 0.0, 0.0, s2)         # 0.234 초과


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok ", t.__name__)
    print(f"{len(tests)} tests passed")
