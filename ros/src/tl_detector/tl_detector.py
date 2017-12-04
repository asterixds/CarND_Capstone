#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import sys
import math
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import scipy.misc


STATE_COUNT_THRESHOLD = 3
HORIZON =200
DEBUG =False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.debug = 0
        self.image_cnt = 0


        self.loop()
        rospy.spin()

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.pose is not None and self.waypoints is not None and self.has_image:
                light_wp, state = self.process_traffic_lights()

                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose
        #rospy.loginfo("position of car is %s", self.pose.position.x)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.loginfo("length of lights %s", len(self.lights))
        #for i, l in enumerate(self.lights):
        #    rospy.loginfo("position of light  %s is %s", i, l.pose.pose.position.x)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        #rospy.loginfo("Received image from simulator")
        self.has_image = True
        self.camera_image = msg

    def distance( self,p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x * x + y * y)


    """get the waypoint closest but ahead of current position"""
    def get_closest_point_idx(self, pose, pose_list):
        def in_front( pose1, pose2):
            def inner_product(A, B):
                return sum([a1 * b1 for a1, b1 in zip(A, B)])

            p1 = pose1.position
            p2 = pose2.position
            wp_vec = (p1.x - p2.x, p2.y - p2.y)
            o = pose.orientation
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
            yaw_vec = (math.cos(yaw), math.sin(yaw))
            return inner_product(wp_vec, yaw_vec) > 0

        next_wp_idx = 0
        closest_dist = float('inf')
        for i, wp in enumerate(pose_list):
            if in_front(wp.pose.pose, pose):
                d = self.distance(wp.pose.pose.position, pose.position)
                if d < closest_dist:
                    closest_dist = d
                    next_wp_idx = i
        return next_wp_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        classification = self.light_classifier.get_classification(cv_image)

        if DEBUG and (int(classification) == TrafficLight.RED):
            self.image_cnt += 1
            scipy.misc.imsave("./images/{}_{}.jpg".format(self.image_cnt, int(classification)), cv_image)
        return classification


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose and self.waypoints):
            #get the light index closest to the current car position
            closest_light_idx = self.get_closest_point_idx(self.pose, self.lights)
            #rospy.logwarn("closest light waypoint is %s", closest_light_idx)
            if self.distance(self.lights[closest_light_idx].pose.pose.position,self.pose.position) <HORIZON:
                state = self.get_light_state(self.lights[closest_light_idx])
                #if state is TrafficLight.GREEN:
                #    rospy.logwarn("Green light detected")
                #if state is TrafficLight.YELLOW:
                #    rospy.logwarn("Yellow light detected")
                if state is TrafficLight.RED:
                    #rospy.logwarn("Red light detected")
                    stop_lines = list()
                    for pos in stop_line_positions:
                        light = TrafficLight()
                        light.pose = PoseStamped()
                        light.pose.pose.position.x = pos[0]
                        light.pose.pose.position.y = pos[1]
                        light.pose.pose.position.z = 0.0
                        stop_lines.append(light)

                    distances = [self.distance(stop_lines[closest_light_idx].pose.pose.position, wp.pose.pose.position)
                                            for wp in self.waypoints]
                    line_wp = distances.index(min(distances))
                    #rospy.logwarn("closest stop line waypoint with red traffic lights %s", self.waypoints[line_wp].pose.pose.position.x)
                    return line_wp, state


        if light:
            state = self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')