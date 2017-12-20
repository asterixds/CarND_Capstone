#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number

MAX_DECEL = 0.5
MPH_TO_MPS = 1609.34/3600.
TARGET_SPEED_MPH = 10.
STOP_BUFFER = 5.0

class WaypointUpdater(object):
    def __init__(self):
        self.current_pose = None
        self.waypoints = None
        self.red_light_waypoint = None

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.publish()
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        #if self.waypoints is not None:
        #    self.publish()

    def waypoints_cb(self, msg):
        if self.waypoints is None: # can also use unregister
            self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.red_light_waypoint = msg.data
        rospy.loginfo("Red Traffic Light ahead at: " + str(msg.data))
        #if self.red_light_waypoint > -1:
        #    self.publish()

    def obstacle_cb(self, msg):
        # TODO
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self,  waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x * x + y * y + z * z)


    def nearest_waypoint(self, pose, waypoints):
        """ for some reason this code fails around the 6000th waypoint - need to check
        distances = [self.distance(pose.position, wp.pose.pose.position)
                             for wp in pose_list]
        next_wp_idx = distances.index(min(distances))"""
        nearest = 100000
        nearest_wp = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)#leaving out z
        for i, wp in enumerate(self.waypoints):
            d = dl(pose.position, wp.pose.pose.position)
            if (d < nearest):
                nearest = d
                nearest_wp = i
        return nearest_wp

    def next_waypoint(self, pose, pose_list):
        # first find nearest point
        next_wp_idx = self.nearest_waypoint(pose, pose_list)
        x = pose_list[next_wp_idx].pose.pose.position.x
        y = pose_list[next_wp_idx].pose.pose.position.y
        p = pose.position
        heading = math.atan2((y - p.y), (x - p.x))
        o = pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))
        angle = abs(yaw - heading)
        # if nearest point is behind then take the next point
        if angle > (math.pi / 4):
            next_wp_idx += 1
        return next_wp_idx % len(self.waypoints)

    def decelerate(self, waypoints, stop_idx):
        if len(waypoints) < 1:
            return []
        stop_idx = min(stop_idx, len(waypoints)-1)
        last = waypoints[stop_idx]
        last.twist.twist.linear.x = 0.
        for i, wp in enumerate(waypoints):
            if i > stop_idx:
                vel = 0.
            else:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position) - STOP_BUFFER
                dist = max(0, dist)
                vel = math.sqrt(2 * MAX_DECEL * dist)
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def publish(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.current_pose is not None and self.waypoints is not None:
                wp0 = self.next_waypoint(self.current_pose, self.waypoints)
                final_waypoints = self.waypoints[wp0:wp0 + LOOKAHEAD_WPS]
                if self.red_light_waypoint is None or self.red_light_waypoint < 0:
                    for i in range(len(final_waypoints) - 1):
                        self.set_waypoint_velocity(final_waypoints[i], TARGET_SPEED_MPH * MPH_TO_MPS)
                else:
                    stop_idx = max(0, self.red_light_waypoint - wp0)
                    # trimming does not work; car wavers at stop light if lookahead is zero
                    # final_waypoints = final_waypoints[:stop_idx]
                    final_waypoints = self.decelerate(final_waypoints, stop_idx)

                # temp code for end of road
                """if wp0 + len(final_waypoints) >= len(self.waypoints):
                    stop_idx = max(0, len(final_waypoints) - wp0)
                    final_waypoints = self.decelerate(final_waypoints, stop_idx)
                    rospy.loginfo("End of road!!!")"""

                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = final_waypoints

                self.final_waypoints_pub.publish(lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')