#! /usr/bin/python
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

## face detection libraries
import cv2
import dlib
import cv_bridge
import json
import math
import time


class LookUpLogicManager:
	#constructor
    def __init__(self):
        self.rate = rospy.Rate(10);
        self.move_pub = rospy.Publisher(
                    '/mobile_base_controller/cmd_vel',
                        geometry_msgs.msg.Twist,
                        latch=True, queue_size=5);
        self.rep_pub = rospy.Publisher(
                    '/bibpoli/lookup_rep',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.cvBridge = cv_bridge.CvBridge();
        self.faceDectector = dlib.get_frontal_face_detector();
        self.state = "STOP";
        self.move_right = geometry_msgs.msg.Twist();
        self.move_right.linear.x = 0;
        self.move_right.linear.y = 0;
        self.move_right.linear.z = 0;
        self.move_right.angular.x = 0;
        self.move_right.angular.y = 0;
        self.move_right.angular.z = -0.3;
        self.move_left = geometry_msgs.msg.Twist();
        self.move_left.linear.x = 0;
        self.move_left.linear.y = 0;
        self.move_left.linear.z = 0;
        self.move_left.angular.x = 0;
        self.move_left.angular.y = 0;
        self.move_left.angular.z = 0.3;
        self.move_stop = geometry_msgs.msg.Twist();
        self.move_stop.linear.x = 0;
        self.move_stop.linear.y = 0;
        self.move_stop.linear.z = 0;
        self.move_stop.angular.x = 0;
        self.move_stop.angular.y = 0;
        self.move_stop.angular.z = 0;
        self.distance_limit = 100;
        self.start = time.time();
        self.time = 2;
        rospy.sleep(3);
        rospy.Subscriber("xtion/rgb/image_rect_color", sensor_msgs.msg.Image, self.image_subscriber_callback_2);
        rospy.Subscriber("bibpoli/lookup/cmd", std_msgs.msg.String, self.command_subscriber);


#    def rect_to_bb(self, rect):
#        x = rect.left();
#        y = rect.top();
#        w = rect.right() - x;
#        h = rect.bottom() - y;
#        return (x, y, w, h);

    def calculate_distance(self, point1x, point1y, point2x, point2y):
        x1 = point1x;
        y1 = point1y;
        x2 = point2x;
        y2 = point2y;
        #return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
        return math.fabs(x1-x2);

    def command_subscriber(self, command):
        self.state = command.data;
        if(self.state == "START"):
            self.centered = False;

    def image_subscriber_callback_1(self, reply):
        if(self.state == "START"):
            frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
            faces_detected = self.faceDectector(frame, 1);
            rospy.loginfo("[image_subscriber_callback] people detected {}".format(len(faces_detected)));
            height, width, channels = frame.shape;
            centerX = width / 2;
            centerY = height / 2;
            min_index = -1;
            min_value = 999999;
            for (i, rect) in enumerate(faces_detected):
                current_distance = self.calculate_distance(centerX, centerY, (rect.left() + rect.right())/2, (rect.bottom() + rect.top())/2);
                if (current_distance <  min_value):
                    min_index = i;
                    min_value = current_distance;
            if(min_index != -1):
                rospy.loginfo("min_index {}, min_value {}".format(min_index, min_value));
                if (min_value > self.distance_limit):
                    rospy.loginfo("is moving");
                    if ( ((faces_detected[min_index].left() + faces_detected[min_index].right())/2) > centerX ):
                        self.move_pub.publish(self.move_right);
                        self.rate.sleep();
                    else:
                        self.move_pub.publish(self.move_left);
                        self.rate.sleep();
                    self.rep_pub.publish("NOCENTER");
                    self.rate.sleep();
                else:
                    self.move_pub.publish(self.move_stop);
                    self.rate.sleep();
                    self.rep_pub.publish("CENTER");
                    self.rate.sleep();
            else:
                self.move_pub.publish(self.move_right);
                self.rate.sleep();


    def image_subscriber_callback_2(self, reply):
        if(self.state == "START"):
            if time.time() - self.start > self.time:
                frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
                faces_detected = self.faceDectector(frame, 1);
                rospy.loginfo("[image_subscriber_callback] people detected {}".format(len(faces_detected)));
                height, width, channels = frame.shape;
                centerX = width / 2;
                centerY = height / 2;
                min_index = -1;
                min_value = 999999;
                for (i, rect) in enumerate(faces_detected):
                    current_distance = self.calculate_distance(centerX, centerY, (rect.left() + rect.right())/2, (rect.bottom() + rect.top())/2);
                    if (current_distance <  min_value):
                        min_index = i;
                        min_value = current_distance;
                if(min_index != -1):
                    rospy.loginfo("min_index {}, min_value {}".format(min_index, min_value));
                    if (min_value > self.distance_limit):
                        rospy.loginfo("is moving");
                        if ( ((faces_detected[min_index].left() + faces_detected[min_index].right())/2) > centerX ):
                            self.move_pub.publish(self.move_right);
                            self.rate.sleep();
                        else:
                            self.move_pub.publish(self.move_left);
                            self.rate.sleep();
                        self.rep_pub.publish("NOCENTER");
                        self.rate.sleep();
                    else:
                        self.move_pub.publish(self.move_stop);
                        self.rate.sleep();
                        self.rep_pub.publish("CENTER");
                        self.rate.sleep();
                else:
                    self.move_pub.publish(self.move_right);
                    self.rate.sleep();
                self.start = time.time();

if __name__ == '__main__':
    rospy.init_node('bibpoli_lookafter_node', anonymous=True);
    try:
        rospy.loginfo("[EXPERIMENT_FACE_DETECT_NODE] STARTED");
        my_logic_manager = LookUpLogicManager();
        rospy.spin();
    except KeyboardInterrupt:
        pass;