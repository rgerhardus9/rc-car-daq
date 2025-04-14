import time

class VelocityProfile:
    def __init__(self, D_TOTAL, V_STEADY, ACCEL):
        self.dTotal = D_TOTAL
        self.vsteady = V_STEADY
        self.accel = ACCEL
        self.t1 = V_STEADY / ACCEL
        self.tsteady = (D_TOTAL / V_STEADY) - (V_STEADY / ACCEL)
        self.t2 = (V_STEADY / ACCEL) + self.tsteady
        self.tfinal = 2 * (V_STEADY / ACCEL) + self.tsteady

    def get_desired_velocity(self, start_time):
        
        # Calculates the desired velocity at the current time based on trapezoidal velocity profile.
        
        now = (time.time() - start_time)
        vd = 0

        # Use an if statement to find out what part of the trapezoid you are in
        if now > self.tfinal:
            vd = 0
        elif now >= self.t2:
            vd = self.vsteady - self.accel * (now - self.t2)
        elif now >= self.t1:
            vd = self.vsteady
        elif now >= 0:
            vd = self.accel * now
        else:
            vd = 0

        print(f"vd: {vd}")
        return vd


    def get_desired_position(self, start_time):
        
        # Calculates the desired position at the current time based on trapezoidal position profile.
        
        now = (time.time() - start_time)
        xd = 0

        # Use an if statement to find out what part of the trapezoid you are in
        if now > self.tfinal:
            xd = self.dTotal
        elif now >= self.t2:
            xd = self.dTotal - 0.5 * self.accel * (self.tfinal - now) ** 2
        elif now >= self.t1:
            xd = 0.5 * self.accel * self.t1 ** 2 + self.vsteady * (now - self.t1)
        elif now >= 0:
            xd = 0.5 * self.accel * now ** 2
        else:
            xd = 0

        return xd
