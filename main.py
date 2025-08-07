from config import Config
from control.robot_interface import RoboticArm
from control.robot_simulator import SimulatedRoboticArm
from control.robot_controller import RobotController
from control.moving_target import MovingTarget
from utils.timing import Clock
from utils.visualizer_pyvista import Visualizer
from utils.transforms import *
from vision.tracker import Tracker
from vision.camera_interface import Camera
import time
import traceback

def main():
    config = Config()
    visualizer = Visualizer()
    print(1)
    # Choose robot interface
    if config.system.mode == "live":
        robot = RoboticArm(config)
    else:
        robot = SimulatedRoboticArm()
    print(2)

    if not robot.setup():
        print("Failed to initialize robot.")
        exit(1)
    print(3)
    
    robot.go_home()

    # ===== Controller and Target =====
    controller = RobotController(robot, config)
    
    if config.system.target == "live":
        cam = Camera(config.vision.cam)
        cam.setup()
        tracker = Tracker(cam, config, "tennis_ball")
    else:
        target = MovingTarget(robot, config)

    # main loop
    clk = Clock(config.robot.target_dt)
    try:
        while True:
            dt = clk.tick()
            tcp_pose = robot.get_tcp_pose()


            if config.system.target == "live":
                cam.update()
                tracker.update()
                target_camf_pos = tracker.get_target_camf_pos()
                target_toolf_pos = target_camf_pos + np.array(config.robot.cam_rel_offset)
                T_basef_tool = pose_to_matrix(tcp_pose)
                target_basef_pos = homo_to_cart(T_basef_tool @ cart_to_homo(target_toolf_pos))
                # print(target_basef_pos - target.pos)
            else:
                target_camf_pos = target.get_target_camf_pos(dt, tcp_pose)
                target_basef_pos = target.pos
            # print(target_camf_pos)

            controller.set_target_camf_pos(target_camf_pos)
            controller.update(dt, tcp_pose)
            
            T_toolf_cam = pose_to_matrix(np.array(list(config.robot.cam_rel_offset) + [0,0,0]))
            T_basef_tool = pose_to_matrix(tcp_pose)
            pose_basef_cam = matrix_to_pose(T_basef_tool @ T_toolf_cam)

            # print(tcp_pose, target_basef_pos, pose_basef_cam)
            visualizer.update(tcp_pose, target_basef_pos, pose_basef_cam)
    except KeyboardInterrupt:
        robot.stop()

    except Exception as e:
        robot.stop()
        print("RTDE connection error:", str(e))
        traceback.print_exc()
        # Try to disconnect cleanly
    
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()