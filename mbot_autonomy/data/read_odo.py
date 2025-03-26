import time
import numpy as np
import lcm
import sys
import os
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.pose2D_t import pose2D_t

odom = []

def my_handler(channel, data):
    print("Recoding odometry... Length: %d" % len(odom))
    msg = pose2D_t.decode(data)
    odom.append([msg.x, msg.y, msg.theta])


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
time.sleep(0.5)
subscription = lc.subscribe("MBOT_ODOMETRY", my_handler)

try:
    while True:
        lc.handle()
        time.sleep(0.05)
except KeyboardInterrupt:
    print("Done")
    # delete the file if it is exist
    try:
        os.remove('odom.txt')
    except OSError:
        pass
    odom = np.array(odom)
    np.savetxt('odom.txt', odom)
    pass