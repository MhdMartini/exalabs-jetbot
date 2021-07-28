"""
Class to receive error values and return [error, error integral, error derrivative]
"""

from collections import deque
import rospy


class PID:
    def __init__(self):
        self.reset()

        self.error_que = deque([], maxlen=2)  # save the last two error values
        self.time_que = deque([], maxlen=2)  # save the times at which errors are received

        self.error_total = 0

    def add_error(self, error) -> list:
        self.error_que.append(error)

        t = rospy.get_rostime().to_sec()
        self.time_que.append(t)

        self.error_total += error

        try:
            d_error = self.error_que[1] - self.error_que[0]
        except IndexError:
            # if we don't have two values yet
            return [error, error, 0]  # p, i, d

        dt = self.time_que[1] - self.time_que[0]
        error_der = d_error / dt
        return [error, self.error_total, error_der]
