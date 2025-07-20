

class SmoothMotionModel:
    def __init__(self, max_speed=0.05, max_rot_speed=0.2):
        self.max_speed = max_speed
        self.max_rot_speed = max_rot_speed

    def compute_velocity(self, current_pose, target_pose, dt):
        # Simple proportional controller for now
        vel = [(t - c) / dt for c, t in zip(current_pose, target_pose)]

        # Clamp velocities
        for i in range(3):
            vel[i] = max(min(vel[i], self.max_speed), -self.max_speed)
        for i in range(3, 6):
            vel[i] = max(min(vel[i], self.max_rot_speed), -self.max_rot_speed)

        return vel
