"""Đạo hàm trên vận tốc đo: alpha=tau/(tau+dt), không lấy đạo hàm setpoint."""
from math import isfinite


class MeasurementDerivative:
    def __init__(self, time_constant):
        if not isfinite(time_constant) or time_constant <= 0:
            raise ValueError('derivative_filter_seconds must be positive and finite')
        self.tau = time_constant
        self.reset()

    def reset(self):
        self.previous = self.stamp = None
        self.value = 0.0
        self.status = 'reset'
        self.alpha = None

    def update(self, measured, stamp_seconds):
        if not isfinite(measured) or not isfinite(stamp_seconds):
            self.reset();self.status='invalid';return self.value
        if self.stamp is not None:
            dt = stamp_seconds-self.stamp
            if dt == 0:
                self.status='duplicate';return self.value
            if dt < 0:
                self.reset();self.status='backwards';return self.value
            if dt > 0.25:
                self.reset();self.status='gap'
            else:
                self.alpha = self.tau/(self.tau+dt)
                self.value = self.alpha*self.value+(1-self.alpha)*(measured-self.previous)/dt
                self.status='fresh'
        else:self.status='initial'
        self.previous,self.stamp=measured,stamp_seconds
        return self.value
