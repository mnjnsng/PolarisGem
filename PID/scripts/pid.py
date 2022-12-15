#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.set_point = set_point

        self.sample_time = sample_time

        self.out_limits = out_limits

        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback_val):
        """Compute PID control value based on feedback_val.
        """

        # TODO: implement PID control
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        current_error = feedback_val
        delta_error = current_error - self.last_err
        error_dot = delta_error/delta_time

        self.p_term = current_error
        self.d_term = error_dot
        self.i_term += current_error * delta_time
        self.last_time = current_time
        self.last_err = current_error

        output = self.Kp*self.p_term + self.Kd*self.d_term + self.Ki*self.i_term
        output = round(np.clip(output,self.out_limits[0],self.out_limits[1]),3)
        return output


    def __call__(self, feeback_val):
        return self.update(feeback_val)
