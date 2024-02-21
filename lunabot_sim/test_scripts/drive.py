#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

# max linear ~0.5
# max angular ~1.5

publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
def publishVelocity(linear, angular, timeDrive):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular

    timeStart = time.time()

    while time.time() < timeStart + timeDrive:
        publisher.publish(msg)
        time.sleep(0.001)

    time.sleep(0.01)

rospy.init_node("manual_node")

time.sleep(4)

# small turns

# publishVelocity(0.5, 0, 3) # forward

# publishVelocity(0.5, -0.5, 5) #right

# publishVelocity(0.5, 0, 3) #forward

# publishVelocity(0.5, 0.5, 5) #left

# publishVelocity(0.5, 0, 3) #forward

# publishVelocity(0, 0, 1) 

# end small turns

#large turns

publishVelocity(0.5, 0, 3) # forward

publishVelocity(0.5, -1, 3) #right

publishVelocity(0.5, 0, 3) #forward

publishVelocity(0.5, 1, 3) #left

publishVelocity(0.5, 0, 3) #forward

publishVelocity(0, 0, 1)

#end large turns

exit()