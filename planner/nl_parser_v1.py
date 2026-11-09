
import argparse
import json
import re
from pathlib import Path

if __package__:
    from .planner_v1 import load_scene, build_plan
else:
    from planner_v1 import load_scene, build_plan


ROOT = Path(__file__).resolve().parents[1]


def parse_instruction(instruction):
    # V1仅支持以逗号分隔的顺序导航指令
    parts = re.split(r"[，,、]", instruction.strip())
    locations = []

    for part in parts:
        part = part.strip().rstrip("。.")

        match = re.fullmatch(
            r"(?:先|再|然后|接着|最后)?\s*"
            r"(?:去|前往|到达|到)\s*"
            r"([\w\u4e00-\u9fff]+区)",
            part
        )

        if not match:
            raise ValueError(
                f"无法识别的导航指令：{part}"
            )

        locations.append(match.group(1))

    return locations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("instruction")
    parser.add_argument(
        "--scene",
        default="turtlebot3_demo"
    )

    args = parser.parse_args()

    try:
        locations = parse_instruction(args.instruction)
        scene = load_scene(args.scene)
        plan = build_plan(scene, locations)
    except ValueError as exc:
        parser.error(str(exc))

    output = ROOT / "plans" / "plan_nl_v1.json"
    output.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    print("自然语言指令解析成功！")
    print("原始指令：", args.instruction)

    for step in plan["steps"]:
        print(
            f"{step['step_id']}. "
            f"{step['action']} → {step['location']}"
        )

    print("计划文件：", output)


if __name__ == "__main__":
    main()
