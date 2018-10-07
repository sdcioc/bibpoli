#! /usr/bin/python
import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg

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
                    '/bibpoli/lookup/rep',
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
        #rospy.Subscriber("xtion/rgb/image_rect_color", sensor_msgs.msg.Image, self.image_subscriber_callback_2);
        rospy.Subscriber("bibpoli/lookup/cmd", std_msgs.msg.String, self.command_subscriber_2);
        self.move_base_actionlib_client = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
        self.move_base_actionlib_client.wait_for_server();


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

    def command_subscriber_1(self, command):
        self.state = command.data;
        if(self.state == "START"):
            self.centered = False;
    


    def command_subscriber_2(self, command):
        self.state = command.data;
        if(self.state == "START"):
            self.centered = False;
            #preiau pozitia curent ma intereseaza doar x y
            reply = None;
            while True:
                try:
                    reply = rospy.wait_for_message(
                    '/amcl_pose',
                    geometry_msgs.msg.PoseWithCovarianceStamped, 3)
                    break;
                except rospy.exceptions.ROSException:
                    rospy.loginfo("[ERROR] No Info from amcl_pose tring again");
            initial_pose = reply.pose.pose;
            initial_pose.orientation.x = 0;
            initial_pose.orientation.y = 0;
            initial_pose.position.z = 0;
            max_people = -1;
            max_index = -1;
            rospy.loginfo(" Strart to move trhough indeces");
            for i in range(0,6):
                teta = i * math.pi / 3.0;
                h = std_msgs.msg.Header();
                h.stamp = rospy.Time.now();
                h.frame_id = "/map";
                initial_pose.orientation.z = math.sin(teta/2);
                initial_pose.orientation.w = math.cos(teta/2);
                move_base_goal = move_base_msgs.msg.MoveBaseGoal();
                move_base_goal.target_pose.header = h;
                move_base_goal.target_pose.pose = initial_pose;
                rospy.loginfo(" Strart to move to {}".format(i*60));
                self.move_base_actionlib_client.send_goal(move_base_goal);
                self.move_base_actionlib_client.wait_for_result(rospy.Duration(5));
                local_max = 0;
                j = 0;
                while(j < 3):
                    try:
                        reply = rospy.wait_for_message(
                        'xtion/rgb/image_rect_color',
                        sensor_msgs.msg.Image, 1);
                        frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
                        faces_detected = self.faceDectector(frame, 1);
                        print "[INFO][check_number_people] people detected {}".format(len(faces_detected));
                        if (len(faces_detected) > local_max):
                            local_max = len(faces_detected);
                    except rospy.exceptions.ROSException:
                        print "[ERROR][check_number_people] No Info from /xtion/rgb/image_rect_color"
                    rospy.sleep(0.5)
                    j = j + 1;
                if (local_max > max_people):
                    max_people = local_max;
                    max_index = i;
            teta = max_index * math.pi / 3.0;
            h = std_msgs.msg.Header();
            h.stamp = rospy.Time.now();
            h.frame_id = "/map";
            initial_pose.orientation.z = math.sin(teta/2);
            initial_pose.orientation.w = math.cos(teta/2);
            move_base_goal = move_base_msgs.msg.MoveBaseGoal();
            move_base_goal.target_pose.header = h;
            move_base_goal.target_pose.pose = initial_pose;
            self.move_base_actionlib_client.send_goal(move_base_goal);
            self.move_base_actionlib_client.wait_for_result();



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