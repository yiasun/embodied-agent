
import argparse
import json
from pathlib import Path
from datetime import datetime, timezone
import sys
if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


if __package__:
    from .validation import MAX_TASKS, MIN_TASKS, strict_json_loads, validate_scene, validate_scene_id
else:
    from validation import MAX_TASKS, MIN_TASKS, strict_json_loads, validate_scene, validate_scene_id


ROOT = Path(__file__).resolve().parents[1]


def load_scene(scene_name):
    validate_scene_id(scene_name)
    directory = (ROOT / "configs" / "scenes").resolve()
    path = (directory / f"{scene_name}.json").resolve()
    if path.parent != directory:
        raise ValueError("场景路径越界")
    if not path.is_file():
        raise ValueError("场景不存在")
    scene = strict_json_loads(path.read_text(encoding="utf-8"))
    validate_scene(scene)
    if scene["scene_id"] != scene_name:
        raise ValueError("场景ID与配置文件名不一致")
    from scenes.assets import validate_scene_assets
    validate_scene_assets(scene, ROOT)
    return scene


def build_plan(scene, locations):
    validate_scene(scene)
    if not isinstance(locations, (list, tuple)) or not MIN_TASKS <= len(locations) <= MAX_TASKS:
        raise ValueError("任务数量必须在1到20之间")
    if any(not isinstance(location, str) for location in locations):
        raise ValueError("地点名称必须为字符串")
    from scenes.assets import validate_scene_assets
    asset_bundle = validate_scene_assets(scene, ROOT)
    steps = []

    for index, location in enumerate(locations, start=1):
        waypoint = scene["waypoints"].get(location)

        if waypoint is None:
            available = ", ".join(scene["waypoints"].keys())
            raise ValueError(
                f"未知地点：{location}，可用地点：{available}"
            )

        x = waypoint.get("x")
        y = waypoint.get("y")

        steps.append({
            "step_id": index,
            "action": "NAVIGATE",
            "location": location,
            "x": float(x),
            "y": float(y)
        })

    plan = {
        "schema_version": "2.0" if asset_bundle else "1.0",
        "scene_id": scene["scene_id"],
        "frame_id": scene["frame_id"],
        "created_at": datetime.now(
            timezone.utc
        ).isoformat(),
        "status": "READY",
        "steps": steps
    }
    if asset_bundle:
        plan['scene_assets'] = asset_bundle['identity']
    return plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "locations",
        nargs="+",
        help="按顺序输入地点名称"
    )
    parser.add_argument(
        "--scene",
        default="turtlebot3_demo"
    )

    args = parser.parse_args()

    try:
        scene = load_scene(args.scene)
        plan = build_plan(scene, args.locations)
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        parser.error(str(exc))

    output = ROOT / "plans" / "plan_v1.json"
    output.parent.mkdir(parents=True, exist_ok=True)

    output.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    print("任务计划生成成功！")
    print(f"场景：{plan['scene_id']}")
    print(f"任务数量：{len(plan['steps'])}")

    for step in plan["steps"]:
        print(
            f"{step['step_id']}. {step['location']} "
            f"→ ({step['x']}, {step['y']})"
        )

    print(f"计划文件：{output}")


if __name__ == "__main__":
    main()
