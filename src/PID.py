import time

class PID:
    def __init__(self, kp, ki, kd, integrator_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integrator, self.prev_error, self.last_ts = 0, 0, time.time()
        self.integrator_max = integrator_max
    def step(self, error):
        now = time.time()
        dt = max(now - self.last_ts, 1e-6)
        self.last_ts = now
        self.integrator = max(min(self.integrator + error * dt, self.integrator_max), -self.integrator_max)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp*error + self.ki*self.integrator + self.kd*derivative
