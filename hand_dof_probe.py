#!/usr/bin/env python3
import asyncio
import json
import math
import time
from collections import defaultdict

import websockets


FINGERS = {
    "thumb": ["thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip"],
    "index": ["index-finger-metacarpal", "index-finger-phalanx-proximal", "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip"],
    "middle": ["middle-finger-metacarpal", "middle-finger-phalanx-proximal", "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip"],
    "ring": ["ring-finger-metacarpal", "ring-finger-phalanx-proximal", "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip"],
    "pinky": ["pinky-finger-metacarpal", "pinky-finger-phalanx-proximal", "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip"],
}


def pos(joints, name):
    item = joints.get(name)
    if not item:
        return None
    p = item.get("pos") or {}
    try:
        return (float(p["x"]), float(p["y"]), float(p["z"]))
    except Exception:
        return None


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(dot(a, a))


def dist(a, b):
    return norm(sub(a, b))


def angle(a, b, c):
    pa, pb, pc = a, b, c
    v1 = sub(pa, pb)
    v2 = sub(pc, pb)
    n1 = norm(v1)
    n2 = norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return None
    value = max(-1.0, min(1.0, dot(v1, v2) / (n1 * n2)))
    return math.degrees(math.acos(value))


def signed_plane_distance(point, origin, normal):
    n = norm(normal)
    if n < 1e-6:
        return None
    return dot(sub(point, origin), normal) / n


def scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def project_to_plane(v, normal):
    n2 = dot(normal, normal)
    if n2 < 1e-9:
        return None
    return sub(v, scale(normal, dot(v, normal) / n2))


def signed_angle_on_plane(v1, v2, normal):
    p1 = project_to_plane(v1, normal)
    p2 = project_to_plane(v2, normal)
    if p1 is None or p2 is None:
        return None
    n1 = norm(p1)
    n2 = norm(p2)
    nn = norm(normal)
    if n1 < 1e-6 or n2 < 1e-6 or nn < 1e-6:
        return None
    unsigned = math.degrees(math.atan2(dot(normal, cross(p1, p2)) / nn, dot(p1, p2)))
    return unsigned


def metrics(side, data):
    joints = data.get("joints") or {}
    fingers = data.get("fingers") or {}
    wrist = pos(joints, "wrist")
    index_meta = pos(joints, "index-finger-metacarpal")
    pinky_meta = pos(joints, "pinky-finger-metacarpal")
    middle_meta = pos(joints, "middle-finger-metacarpal")
    thumb_meta = pos(joints, "thumb-metacarpal")
    thumb_prox = pos(joints, "thumb-phalanx-proximal")
    thumb_distal = pos(joints, "thumb-phalanx-distal")
    thumb_tip = pos(joints, "thumb-tip")
    index_tip = pos(joints, "index-finger-tip")

    out = {"joint_count": len(joints), "finger_keys": sorted(fingers.keys())}
    if wrist and thumb_tip:
        out["thumb_wrist_dist"] = dist(thumb_tip, wrist)
    if thumb_tip and index_tip:
        out["pinch_dist"] = dist(thumb_tip, index_tip)
    if thumb_meta and thumb_prox and thumb_distal:
        out["thumb_mcp_angle"] = angle(thumb_meta, thumb_prox, thumb_distal)
    if thumb_prox and thumb_distal and thumb_tip:
        out["thumb_ip_angle"] = angle(thumb_prox, thumb_distal, thumb_tip)
    if wrist and index_meta and pinky_meta and thumb_tip:
        palm_normal = cross(sub(index_meta, wrist), sub(pinky_meta, wrist))
        out["thumb_palm_plane"] = signed_plane_distance(thumb_tip, wrist, palm_normal)
        if middle_meta and thumb_meta:
            palm_forward = sub(middle_meta, wrist)
            thumb_axis = sub(thumb_tip, thumb_meta)
            out["thumb_spread_angle"] = signed_angle_on_plane(palm_forward, thumb_axis, palm_normal)
    if wrist and middle_meta and thumb_tip:
        out["thumb_forward_dist"] = dist(thumb_tip, middle_meta)

    bends = {}
    for name, chain in FINGERS.items():
        pts = [pos(joints, j) for j in chain]
        vals = []
        for i in range(1, len(pts) - 1):
            if pts[i - 1] and pts[i] and pts[i + 1]:
                vals.append(angle(pts[i - 1], pts[i], pts[i + 1]))
        if vals:
            bends[name] = round(sum(vals) / len(vals), 1)
    out["bend_angles_deg"] = bends
    return out


def numeric_metrics(m):
    out = {}
    for key in (
        "thumb_wrist_dist",
        "pinch_dist",
        "thumb_mcp_angle",
        "thumb_ip_angle",
        "thumb_spread_angle",
        "thumb_palm_plane",
        "thumb_forward_dist",
    ):
        if m.get(key) is not None:
            out[key] = float(m[key])
    for name, value in (m.get("bend_angles_deg") or {}).items():
        out[f"bend_{name}"] = float(value)
    return out


def add_sample(stats, label, side, m):
    bucket = stats[(label, side)]
    bucket["count"] += 1
    for key, value in numeric_metrics(m).items():
        item = bucket["values"][key]
        item["sum"] += value
        item["min"] = value if item["min"] is None else min(item["min"], value)
        item["max"] = value if item["max"] is None else max(item["max"], value)


def print_summary(stats):
    print("\n" + "#" * 80)
    print("POSE SUMMARY")
    for (label, side), bucket in sorted(stats.items()):
        count = bucket["count"]
        if count == 0:
            continue
        parts = []
        for key in (
            "thumb_wrist_dist",
            "pinch_dist",
            "thumb_mcp_angle",
            "thumb_ip_angle",
            "thumb_spread_angle",
            "thumb_palm_plane",
            "thumb_forward_dist",
            "bend_thumb",
            "bend_index",
            "bend_middle",
            "bend_ring",
            "bend_pinky",
        ):
            item = bucket["values"].get(key)
            if not item:
                continue
            mean = item["sum"] / count
            parts.append(f"{key}={mean:.3f}[{item['min']:.3f},{item['max']:.3f}]")
        print(f"{label:14s} {side:5s} n={count:4d} " + " ".join(parts))
    print("#" * 80 + "\n")


async def handler(websocket):
    print("WebSocket client connected")
    stats = defaultdict(lambda: {"count": 0, "values": defaultdict(lambda: {"sum": 0.0, "min": None, "max": None})})
    last = 0.0
    last_summary = time.time()
    async for message in websocket:
        now = time.time()
        try:
            data = json.loads(message)
        except json.JSONDecodeError as exc:
            print(f"bad json: {exc}")
            continue

        label = data.get("pose_label", "unlabeled")
        for side in ("left", "right"):
            hand = data.get(side) or {}
            if not hand:
                continue
            add_sample(stats, label, side, metrics(side, hand))

        if now - last < 1.0:
            if now - last_summary > 30.0:
                print_summary(stats)
                last_summary = now
            continue
        last = now

        print("=" * 80)
        print(f"pose={label} name={data.get('pose_name')} remaining={float(data.get('pose_remaining', 0.0)):.1f}s")
        for side in ("left", "right"):
            hand = data.get(side) or {}
            if not hand:
                print(f"{side}: missing")
                continue
            m = metrics(side, hand)
            print(f"{side}: joints={m['joint_count']} finger_keys={m['finger_keys']}")
            print(
                "  thumb wrist={:.4f} pinch={:.4f} mcp={} ip={} spread_angle={} palm_plane={} forward={} bends={}".format(
                    m.get("thumb_wrist_dist", float("nan")),
                    m.get("pinch_dist", float("nan")),
                    None if m.get("thumb_mcp_angle") is None else round(m["thumb_mcp_angle"], 1),
                    None if m.get("thumb_ip_angle") is None else round(m["thumb_ip_angle"], 1),
                    None if m.get("thumb_spread_angle") is None else round(m["thumb_spread_angle"], 1),
                    None if m.get("thumb_palm_plane") is None else round(m["thumb_palm_plane"], 4),
                    None if m.get("thumb_forward_dist") is None else round(m["thumb_forward_dist"], 4),
                    m.get("bend_angles_deg", {}),
                )
            )

        if now - last_summary > 30.0:
            print_summary(stats)
            last_summary = now


async def main():
    print("Listening on ws://0.0.0.0:8765")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
