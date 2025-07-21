from config import Config
from control.robot_interface import RoboticArm
from control.robot_simulator import SimulatedRoboticArm
from control.robot_controller import RobotController
from control.moving_target import MovingTarget
from utils.timing import Clock
from utils.visualizer import Visualizer


def main():
    config = Config()
    visualizer = Visualizer()

    # Choose robot interface
    if config.system.mode == "live":
        robot = RoboticArm(config.robot.ip)
    else:
        robot = SimulatedRoboticArm()

    if not robot.setup():
        print("Failed to initialize robot.")
        exit(1)

    # Setup controller and timer
    controller = RobotController(robot, config)
    controller.set_desired_pose([0, 0, 0.5, 0, 0, 0]) # TODO: tmp - read from config

    clk = Clock(config.robot.target_dt)

    # Optionally add a moving target - IF SIM
    target = MovingTarget(robot) # cam later

    try:
        while True:
            dt = clk.tick()
            target_pose = target.get_target_pose(dt)
            controller.set_target_pose(target_pose)  # You may need to implement this
            controller.update(dt)
            visualizer.update(robot.get_tcp_pose(), target_pose + robot.get_tcp_pose())
    except KeyboardInterrupt:
        robot.stop()


if __name__ == "__main__":
    main()