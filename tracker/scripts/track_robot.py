from ball_tracking import BallTracker
from filtered_objects import Sphero, Light
from filtered_objects import TennisBall

if __name__ == '__main__':
    rb = TennisBall()
    sb = Sphero()
    lb = Light()
    bt = BallTracker(objects_to_track=[sb,lb])
    #bt = BallTracker()
    try:
        bt.track()
    except KeyboardInterrupt:
        bt.stop_track()
