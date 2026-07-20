"""Build a Foxglove layout matching what the episode logger records.

Foxglove opens a file with whatever layout is currently selected, so a fresh
install shows panels bound to topics that do not exist in our recordings. This
generates a layout whose panels point at the topics actually present in an
episode, and installs it into Foxglove's local layout store.

Panels are derived from the file, so a second camera added to
``env.robot.cameras`` gets its own Image panel automatically.

Schema notes (verified against Foxglove Studio 2.57): a layout is a plain JSON
file under ``~/.config/Foxglove/studio-datastores/layouts-local/<id>``, where the
panel tree lives in ``baseline.data.layout`` (a nested mosaic of
``{direction, first, second, splitPercentage}``) and per-panel settings in
``baseline.data.configById``, keyed ``"<PanelType>!<suffix>"``.
"""

from __future__ import annotations

import datetime as _dt
import json
import os

LAYOUT_ID = "lay_piperhilserl0001"  # fixed, so re-installing updates in place
LAYOUT_NAME = "HIL-SERL Transitions"

# Foxglove's Image panel is 3D-backed and expects these alongside `imageMode`.
_IMAGE_DEFAULTS = {
    "cameraState": {
        "distance": 20,
        "perspective": True,
        "phi": 60,
        "target": [0, 0, 0],
        "targetOffset": [0, 0, 0],
        "targetOrientation": [0, 0, 0, 1],
        "thetaOffset": 45,
        "fovy": 45,
        "near": 0.5,
        "far": 5000,
    },
    "followMode": "follow-pose",
    "scene": {},
    "transforms": {},
    "topics": {},
    "layers": {},
}

_PLOT_COLOURS = ["#4e98e2", "#f5774d", "#a0d468", "#ed5565", "#ac92ec"]


def _image_panel(topic: str) -> dict:
    return {**_IMAGE_DEFAULTS, "imageMode": {"imageTopic": topic}}


def _plot_panel(paths: list[str]) -> dict:
    return {
        "paths": [
            {
                "timestampMethod": "receiveTime",
                "value": path,
                "enabled": True,
                "color": _PLOT_COLOURS[i % len(_PLOT_COLOURS)],
            }
            for i, path in enumerate(paths)
        ],
        "showXAxisLabels": True,
        "showYAxisLabels": True,
        "showLegend": True,
        "legendDisplay": "floating",
        "showPlotValuesInLegend": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
    }


def _raw_panel(topic: str) -> dict:
    return {
        "diffEnabled": False,
        "diffMethod": "custom",
        "diffTopicPath": "",
        "showFullMessageForDiff": False,
        "topicPath": topic,
        "fontSize": 12,
    }


def _column(panels: list[str]) -> object:
    """Nest panel ids into a top-to-bottom mosaic."""
    if not panels:
        return None
    if len(panels) == 1:
        return panels[0]
    return {
        "direction": "column",
        "first": panels[0],
        "second": _column(panels[1:]),
        "splitPercentage": 100.0 / len(panels),
    }


def _safe(topic: str) -> str:
    return "".join(c for c in topic if c.isalnum()) or "panel"


def build_layout(topics: list[str]) -> dict:
    """Layout JSON for the given topic list (as recorded in an episode)."""
    available = set(topics)
    config: dict[str, dict] = {}
    left: list[str] = []

    # Left column: every camera, then the grasp overlay beneath them.
    for topic in sorted(t for t in available if t.startswith("/camera/")):
        pid = f"Image!{_safe(topic)}"
        config[pid] = _image_panel(topic)
        left.append(pid)
    if "/grasp/overlay" in available:
        pid = "Image!graspoverlay"
        config[pid] = _image_panel("/grasp/overlay")
        left.append(pid)

    # Right column: reward/intervention plot, then the numeric detail panels.
    right: list[str] = []
    plot_paths = [p for p in ("/reward.reward", "/reward.episode_return", "/control.value")
                  if p.split(".")[0] in available]
    if plot_paths:
        config["Plot!rewards"] = _plot_panel(plot_paths)
        right.append("Plot!rewards")
    for topic in ("/state", "/action", "/grasp"):
        if topic in available:
            pid = f"RawMessages!{_safe(topic)}"
            config[pid] = _raw_panel(topic)
            right.append(pid)

    left_tree, right_tree = _column(left), _column(right)
    if left_tree and right_tree:
        tree: object = {
            "direction": "row",
            "first": left_tree,
            "second": right_tree,
            "splitPercentage": 45,
        }
    else:
        tree = left_tree or right_tree or "RawMessages!empty"
        if tree == "RawMessages!empty":
            config[tree] = _raw_panel("/state")

    return {
        "id": LAYOUT_ID,
        "name": LAYOUT_NAME,
        "permission": "CREATOR_WRITE",
        "baseline": {
            "data": {
                "configById": config,
                "globalVariables": {},
                "userNodes": {},
                "playbackConfig": {"speed": 1},
                "layout": tree,
            },
            "savedAt": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
        },
    }


#: Everything the logger publishes besides the per-camera image topics.
BASE_TOPICS = ["/grasp/overlay", "/state", "/action", "/reward", "/control", "/grasp", "/episode"]


def topics_from_config(config_path: str) -> list[str]:
    """Topic list implied by an RL config.

    Used for the **live** stream, where there is no recording to inspect yet: the
    camera panels come from ``env.robot.cameras`` so the layout is right from the
    first frame.
    """
    with open(config_path) as handle:
        cfg = json.load(handle)
    cameras = (cfg.get("env", {}).get("robot", {}) or {}).get("cameras", {}) or {}
    return sorted([f"/camera/{name}" for name in cameras] + BASE_TOPICS)


def topics_in(mcap_path: str) -> list[str]:
    from mcap.reader import make_reader

    with open(mcap_path, "rb") as handle:
        summary = make_reader(handle).get_summary()
        return sorted({ch.topic for ch in summary.channels.values()})


def local_layout_dir() -> str:
    return os.path.expanduser("~/.config/Foxglove/studio-datastores/layouts-local")


def install(layout: dict, target_dir: str | None = None) -> str:
    """Write the layout into Foxglove's local store; returns the path written."""
    directory = target_dir or local_layout_dir()
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, layout["id"])
    with open(path, "w") as handle:
        json.dump(layout, handle)
    return path


def _main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Generate/install the HIL-SERL Foxglove layout")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--config", help="RL config json (topics inferred from env.robot.cameras)")
    src.add_argument("--episode", help="recorded .mcap (topics read from the file)")
    ap.add_argument("--dir", help="install dir (default: Foxglove's local layout store)")
    args = ap.parse_args()

    topics = topics_from_config(args.config) if args.config else topics_in(args.episode)
    path = install(build_layout(topics), args.dir)
    print(f'installed layout "{LAYOUT_NAME}" -> {path}')
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
