"""只读手柄探针：订阅 SensorJoy 并打印原始 axes/buttons，不下发任何指令。

用法: python3 tools/joy_probe.py <robot-ip>   (默认 127.0.0.1，即本机仿真)
"""
import sys
import time

from limxsdk.robot import Robot, RobotType

# 与 WheelfootController.JOY_BTNS / JOY_AXES 保持一致
EXPECTED_BTNS = {"×": 0, "○": 1, "□": 2, "△": 3, "L1": 4, "R1": 5}
EXPECTED_AXES = {"left_vertical": 1, "left_horizon": 0, "right_horizon": 2}

state = {"count": 0, "last_sig": None, "first": 0.0}


def on_joy(joy):
    axes = list(joy.axes)
    buttons = list(joy.buttons)
    state["count"] += 1
    if state["first"] == 0.0:
        state["first"] = time.time()
        print(f"首帧到达: {len(axes)} 轴, {len(buttons)} 按钮")

    pressed = [i for i, b in enumerate(buttons) if b]
    sig = (tuple(pressed), tuple(round(a, 2) for a in axes))
    if sig == state["last_sig"]:
        return
    state["last_sig"] = sig

    names = [n for n, i in EXPECTED_BTNS.items() if i in pressed]
    combo = ""
    if EXPECTED_BTNS["L1"] in pressed:
        if EXPECTED_BTNS["○"] in pressed:
            combo = "  <== L1+○ 会触发 prepare"
        elif EXPECTED_BTNS["△"] in pressed:
            combo = "  <== L1+△ 会触发 start_controller"
        elif EXPECTED_BTNS["×"] in pressed:
            combo = "  <== L1+× 会触发 stop_controller"
    print(f"#{state['count']:<6} pressed={pressed} {names}  "
          f"axes={[round(a, 2) for a in axes]}{combo}")


if __name__ == '__main__':
    ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    robot = Robot(RobotType.Tron2)
    if not robot.init(ip):
        sys.exit("robot.init failed")
    robot.subscribeSensorJoy(on_joy)

    print(f"监听 SensorJoy @ {ip}，请依次单独按 L1 / X / Y / R1 并推摇杆。Ctrl+C 退出。")
    print(f"控制器期望的映射: {EXPECTED_BTNS}, axes {EXPECTED_AXES}")
    try:
        while True:
            time.sleep(1.0)
            if state["count"] == 0:
                print("...尚未收到任何 SensorJoy 数据")
    except KeyboardInterrupt:
        print(f"\n共收到 {state['count']} 帧 SensorJoy")
