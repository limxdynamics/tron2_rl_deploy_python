import argparse
import os
import sys

import controllers

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="TRON2 policy controller entry")
    parser.add_argument("robot_ip", nargs="?", default="127.0.0.1",
                        help="robot ip (default 127.0.0.1)")
    parser.add_argument("--sdk", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true",
                        help="只校验并打印关节指令，不向机器人下发")
    args = parser.parse_args()

    # Get the robot type from the environment variable
    robot_type = os.getenv("ROBOT_TYPE")

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("\033[31mError: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.\033[0m")
        sys.exit(1)

    model_dir = f'{os.path.dirname(os.path.abspath(__file__))}/controllers/model'

    if robot_type not in ("SF_TRON2A", "WF_TRON2A", "DASF_TRON2A"):
        print(f"\033[31mError: unsupported ROBOT_TYPE='{robot_type}', expected SF_TRON2A, WF_TRON2A, or DASF_TRON2A\033[0m")
        sys.exit(1)

    # DA-SF uses the Centaur SDK for motion, IMU, and joystick channels.
    if robot_type == "DASF_TRON2A":
        os.environ.setdefault("ROBOT_IP", args.robot_ip)
        controller = controllers.DASFController(model_dir, robot_type, False)
        controller.run()
        sys.exit(0)

    # Create a Robot instance of the specified type
    from limxsdk.robot import Robot, RobotType
    robot = Robot(RobotType.Tron2)

    # Initialize the robot with the provided IP address
    if not robot.init(args.robot_ip):
        sys.exit()

    if args.dry_run:
        from controllers.dry_run import DryRunRobot
        robot = DryRunRobot(robot, f'{model_dir}/{robot_type}/params.yaml')
        print("\033[33m[DRY-RUN] 只校验并打印关节指令，不会向机器人下发任何指令。\033[0m")

    use_pygame_joystick = True

    # Determine if the simulation is running
    start_controller = False

    # Create and run the controller
    if robot_type == "SF_TRON2A":
        controller = controllers.SolefootController(model_dir, robot, robot_type, start_controller, use_pygame_joystick=use_pygame_joystick)
        controller.run()
    elif robot_type == "WF_TRON2A":
        controller = controllers.WheelfootController(model_dir, robot, robot_type, start_controller, use_pygame_joystick=use_pygame_joystick)
        controller.run()
