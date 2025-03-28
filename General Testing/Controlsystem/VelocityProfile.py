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
        self.start_time = None
        self.executed = False

    def get_desired_velocity(self):

        #This function only runs once for the first itteration
        if not self.executed:       
            self.executed = True
            self.start_time = time.time()


        # Calculates the desired velocity at the current time based on trapezoidal velocity profile.
        now = (time.time() - self.start_time)
        vd = 0
        print(f"start Time : {self.start_time}")
        print(f"Time: {time.time()}")
        print(f"Now:  {now}")

        # Use an if statement to find out what part of the trapezoid you are in
        if self.tfinal < now:
            vd = 0
        elif self.t2 < now <= self.tfinal:
            vd = self.vsteady - self.accel * (now - self.t2)
        elif self.t1 < now <= self.t2:
            vd = self.vsteady
        elif 0 <= now <= self.t1:
            vd = self.accel * now
        else:
            vd = 0

        return vd


    def get_desired_position(self):
        # Calculates the desired position at the current time based on trapezoidal position profile.
        #This function only runs once for the first itteration
        if not self.executed:       
            self.executed = True
            self.start_time = time.time()
        
        now = (time.time() - self.start_time)
        xd = 0
        # Use an if statement to find out what part of the trapezoid you are in
        if self.tfinal < now:
            xd = self.dTotal
        elif self.t2 < now <= self.tfinal:
            xd = self.dTotal - 0.5 * self.accel * (self.tfinal - now) ** 2
        elif self.t1 < now <= self.t2 :
            xd = 0.5 * self.accel * self.t1 ** 2 + self.vsteady * (now - self.t1)
        elif 0 <= now <= self.t1:
            xd = 0.5 * self.accel * now ** 2
        else:
            xd = 0

        return xd
