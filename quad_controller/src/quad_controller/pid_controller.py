# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        #Initialization
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        self.max_windup_ = float(max_windup)

        #Store relevant data
        self.last_timestamp_=0.0
        self.set_point_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0
        self.last_windup_ = 0.0
        self.u_p =[0]
        self.u_i =[0]
        self.u_d = [0]

    def reset(self):
        #clear class variables
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0
        self.last_windup_ = 0.0

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKI(self, ki):
        self.ki_ = float(ki)

    def setKD(self, kd):
        self.kd_ = kd

    def setMaxWindup(self, max_windup):
        self.max_windup_ = float(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            return 0

        #calculate the error
        error = self.set_point_ - float(measured_value)
        self.error_sum_ += error * delta_time
        delta_error = error - self.last_error_

        #Adress max windup
        if self.error_sum_ > self.max_windup_ :
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_ :
            self.error_sum_ = -self.max_windup_

        #proportianal part
        p = self.kp_ * error
        #integration part
        i = self.ki_ * self.error_sum_
        #derivative part
        d = self.kd_ * (delta_error / delta_time)

        #control effort
        u = p +i +d
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)             

        self.last_timestamp_ = timestamp
        self.last_error_ = error

        return u

        


