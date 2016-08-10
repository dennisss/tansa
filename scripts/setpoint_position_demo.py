
import rospy
import thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler


class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self.reached)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(10)   # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)

            self.pub.publish(msg)
            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.01

        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()


def setpoint_demo():
    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)

    setpoint = SetpointPosition()

    #rospy.loginfo("Climb")
    #setpoint.set(0.0, 0.0, 3.0, 0)
    #setpoint.set(0.0, 0.0, 10.0, 5)

    #rospy.loginfo("Sink")
    #setpoint.set(0.0, 0.0, 8.0, 5)

    #rospy.loginfo("Fly to the right")
    #setpoint.set(10.0, 4.0, 8.0, 5)

    #rospy.loginfo("Fly to the left")
    #setpoint.set(0.0, 0.0, 8.0, 5)

    offset_x = 0.0
    offset_y = 0.0
    offset_z = 1.0
    sides = 360
    radius = 1

    rospy.loginfo("Fly in a circle")
    setpoint.set(0.0, 0.0, 1.0, 3)   # Climb to the starting height first
    i = 0
    while not rospy.is_shutdown():
        x = radius * cos(i * 2 * pi / sides) + offset_x
        y = radius * sin(i * 2 * pi / sides) + offset_y
        z = offset_z

        wait = False
        delay = 0
        if (i == 0 or i == sides):
            # Let it reach the setpoint.
            wait = True
            delay = 5

        setpoint.set(x, y, z, delay, wait)

        i = i + 1
        rate.sleep()

        if (i > sides):
            rospy.loginfo("Fly home")
            setpoint.set(0.0, 0.0, 1.0, 5)
            break

    # Simulate a slow landing.
    #setpoint.set(0.0, 0.0,  8.0, 5)
    #setpoint.set(0.0, 0.0,  3.0, 5)
    #setpoint.set(0.0, 0.0,  2.0, 2)
    #setpoint.set(0.0, 0.0,  1.0, 2)
    #setpoint.set(0.0, 0.0,  0.0, 2)
    #setpoint.set(0.0, 0.0, -0.2, 2)

    rospy.loginfo("Bye!")


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
