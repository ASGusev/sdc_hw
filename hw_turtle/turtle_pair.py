#! /usr/bin/python

import math
from collections import namedtuple

import rospy
import roslaunch
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist


Point2D = namedtuple('Point2D', 'x y')


class Vec2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def angle(self):
        return math.atan2(self.y, self.x)

    @classmethod
    def by_points(cls, src, tgt):
        return Vec2D(tgt.x - src.x, tgt.y - src.y)


ANGLE_SPEED_MULTIPLIER = 10
SPEED_DISTANCE_THRESHOLD = 1.
SPEED_ANGLE_THRESHOLD = math.pi / 2


class TurtlePair:
    def __init__(self, max_follower_speed):
        self.make_board()
        self.leader_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.on_leader_update)
        self.follower_pose_subscriber = rospy.Subscriber('/turtle2/pose', Pose, self.on_follower_update)
        self.follower_cmd_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=16)
        self.leader_pos = None
        self.follower_pos = None
        self.follower_theta = None
        self.max_follower_speed = max_follower_speed

    def on_leader_update(self, new_pose):
        self.leader_pos = Point2D(new_pose.x, new_pose.y)
        self.update_follower()

    def on_follower_update(self, new_pose):
        self.follower_pos = Point2D(new_pose.x, new_pose.y)
        self.follower_theta = new_pose.theta
        self.update_follower()

    def calc_linear_speed(self, target_vector_norm):
        if target_vector_norm >= SPEED_DISTANCE_THRESHOLD:
            return self.max_follower_speed
        return self.max_follower_speed * target_vector_norm / SPEED_DISTANCE_THRESHOLD

    def calc_target_angle(self, target_vector_argument):
        diff = target_vector_argument - self.follower_theta
        if diff > math.pi:
            diff = 2 * math.pi - diff
        elif diff < -math.pi:
            diff = 2 * math.pi + diff
        return diff

    def update_follower(self):
        if self.leader_pos is not None and self.follower_pos is not None:
            target_vector = Vec2D.by_points(self.follower_pos, self.leader_pos)
            msg = Twist()
            target_angle = self.calc_target_angle(target_vector.angle())
            msg.angular.z = target_angle * ANGLE_SPEED_MULTIPLIER
            if abs(target_angle) <= SPEED_ANGLE_THRESHOLD:
                msg.linear.x = self.calc_linear_speed(target_vector.norm())
            else:
                msg.linear.x = 0.
            self.follower_cmd_publisher.publish(msg)

    @staticmethod
    def make_board():
        turtle_node = roslaunch.core.Node('turtlesim', 'turtlesim_node')
        launcher = roslaunch.scriptapi.ROSLaunch()
        launcher.start()
        launcher.launch(turtle_node)

        rospy.wait_for_service('/spawn')
        spawn_func = rospy.ServiceProxy('/spawn', Spawn)
        spawn_func(1.0, 1.0, 0.0, 'turtle2')


rospy.init_node('turtle_pair')
follower_speed = float(rospy.get_param('/turtle_pair/speed'))
turtles_pair = TurtlePair(follower_speed)

rate = rospy.Rate(1.)
while not rospy.is_shutdown():
    rate.sleep()
