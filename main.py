from config import Config
from control.robot_interface import RoboticArm
from control.robot_simulator import SimulatedRoboticArm
from control.robot_controller import RobotController
from control.moving_target import MovingTarget
from utils.timing import Clock
from utils.visualizer_pyvista import Visualizer
from utils.transforms import apply_relative_pose
from vision.tracker import Tracker
from vision.camera_interface import Camera
import time

def main():
    config = Config()
    visualizer = Visualizer()
    # visualizer2 = Visualizer() # for camera view

    # ==== vision ====
    # camera = Camera(config.vision.cam)
    # camera.setup()

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
    controller.set_desired_rel_pose(config.robot.default_desired_rel_pose)

    clk = Clock(config.robot.target_dt)

    # Optionally add a moving target - IF SIM
    target = MovingTarget(robot) # cam later
    # target = Tracker(camera)

    try:
        while True:
            dt = clk.tick()

            # camera.update()
            # target.update()
            
            target_rel_pose = target.get_target_rel_pose(dt)
            controller.set_target_rel_pose(target_rel_pose)
            controller.update(dt)

            visualizer.update(robot.get_tcp_pose(), apply_relative_pose(robot.get_tcp_pose(), target_rel_pose))
            # visualizer2.update(config.robot.default_desired_rel_pose, target_rel_pose)
    except KeyboardInterrupt:
        robot.stop()


if __name__ == "__main__":
    main()