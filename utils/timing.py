import time

class Clock:
    def __init__(self, target_dt):
        """
        :param target_dt: Desired time between ticks (in seconds), e.g., 0.01 for 100Hz
        """
        self.target_dt = target_dt
        self.last_time = time.time()

    def tick(self):
        """
        Waits the remaining time to match the target_dt since the last tick,
        then returns the actual time passed (in seconds).
        """
        now = time.time()
        elapsed = now - self.last_time
        wait_time = max(0.0, self.target_dt - elapsed)

        if wait_time > 0:
            time.sleep(wait_time)
        else:
            print("Overclocking!", self.target_dt - elapsed, flush=False)

        current_time = time.time()
        actual_dt = current_time - self.last_time
        self.last_time = current_time
        return actual_dt
