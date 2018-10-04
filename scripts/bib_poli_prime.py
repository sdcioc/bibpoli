#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018


import math
import sys
import json
import copy

import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import pal_web_msgs.msg
import std_srvs.srv

## face detection libraries
import cv2
import dlib
import cv_bridge

#sound
import subprocess
import random
import rospkg

def convert_POIPosition_MapPosition(position):
	#tipul de mesaj pentru map
	pos = geometry_msgs.msg.PoseStamped();
	#harta pe care are loc pozitionarea si directia
	pos.header.frame_id = 'map';
	pos.pose.position.x = position[0];
	pos.pose.position.y = position[1];
	pos.pose.orientation.z = math.sin(position[2] / 2.0);
	pos.pose.orientation.w = math.cos(position[2] / 2.0);
	return pos;

#intrare geometry_msgs.msg.PoseStamped (vector3 position, vector3 orientation)
def convert_MapPosition_POIPosition(position):
        x = position.position.x;
        y = position.position.y;
        w = 2 * math.acos(position.orientation.w);
        return (x, y, w);

def convert_POIName_RosparamName(poi_name):
	prefix = '/mmap/poi/submap_0/';
	if not poi_name.startswith(prefix):
		poi_name = prefix + poi_name;
	return poi_name;

class POILocationManager:
	#constructor
	def __init__(self):
		self.prefix = '/mmap/poi/submap_0/';

	#returneaza pozitia pe harta a punctului de interes
	def get_position(self, poi_name):
		print "[INFO] getting position for poi {}".format(poi_name) 
		poi_name = convert_POIName_RosparamName(poi_name)
		try:
			poi = rospy.get_param(poi_name)
			if not poi:
				return None
			if len(poi[2:]) != 3:
				return None
			position = convert_POIPosition_MapPosition(poi[2:])
			return position
		except KeyError:
			return None


#Clasa ce ofera informatiile despre puncte de interes din camere
class POIInfoManager:
    #constructor
    def __init__(self, filename):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = filename;
        with open(self.filename) as fd:
            self.points = json.loads(fd.read());
            self.current = 0;

    # returneanza numele punctului de interes curent
    def get_poi_name(self):
        return self.points[self.current]['poi_name'];

    # trece la urmatoare camera nevizitata
    def next_poi(self):
        self.current = (self.current + 1) % len(self.points);

    def set_poi(self, poi_name):
        for point in self.points:
            if (poi_name == point['poi_name']):
                self.current = self.points.index(point);
                return True;
        return False;

class SoundManager:
    def __init__(self):
        self.utility = "aplay ";
        self.general_volume_param = "/pal/general_volume";
        self.playback_volume_param = "/pal/playback_volume";
        rospy.set_param(self.general_volume_param, 80);
        rospy.set_param(self.playback_volume_param, 80);
        rospack = rospkg.RosPack();
        self.prefix = rospack.get_path('bib_poli_package') + "/config/sounds/";
    
    def play_introduction(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "1";
        else:
            command = command + "2";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + "wav";
        subprocess.check_output(command.split());


    def play_ensta_description(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "3";
        elif(message_type == 1):
            command = command + "4";
        else:
            command = command + "5";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + "wav";
        subprocess.check_output(command.split());
    
    def play_road_to_ensta(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "6";
        else:
            command = command + "7";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + "wav";
        subprocess.check_output(command.split());
    
    def play_thank_you(self, speed):
        command = self.utility + self.prefix;
        command = command + "11";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + "wav";
        subprocess.check_output(command.split());


    def play_let_me_pass(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "8";
        elif(message_type == 1):
            command = command + "9";
        else:
            command = command + "10";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + "wav";
        subprocess.check_output(command.split());
    

class ExperimentLogicManager:
	#constructor
    def __init__(self, infofilename):
        self.poi_info_manager = POIInfoManager(infofilename);
        self.poi_location_manager = POILocationManager();
        self.sound_manager = SoundManager();
        self.state = "INITIAL";
        self.DISTANCE_ERROR = 0.6;
        self.sonar_param = "/speed_limit/limitess/base_sonars/obstacle_max/dist";
        self.laser_param = "/speed_limit/limitess/base_laser/obstacle_max/dist";
        self.obstacle_normal_dist = 0.3;
        self.obstacle_door_dist = 0.05;
        self.people_number = 0;
        self.rate = rospy.Rate(10);
        self.tries = 0;
        self.poi_name = None;
        self.poi_position = None;
        self.command_pub = rospy.Publisher(
                    '/bibpoli/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.move_pub = rospy.Publisher(
                    '/move_base_simple/goal',
                        geometry_msgs.msg.PoseStamped,
                        latch=True, queue_size=5);
        self.web_pub = rospy.Publisher(
                    '/web/go_to',
                        pal_web_msgs.msg.WebGoTo,
                        latch=True, queue_size=5);
        self.look_after_pub = rospy.Publisher(
                    '/bibpoli/lookup_cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        rospy.sleep(3);
        rospy.Subscriber("bibpoli/cmd", std_msgs.msg.String, self.command_subscriber_callback);
        rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.position_subscriber_callback);
        rospy.sleep(3);
        ## face_detec
        self.cvBridge = cv_bridge.CvBridge();
        self.faceDectector = dlib.get_frontal_face_detector();
        self.last_point = None;
        rospy.wait_for_service('move_base/clear_costmaps');
        self.clear_costmaps = clear_bg = rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty);

    def command_subscriber_callback(self, ros_string_command):
        rospy.loginfo("[COMMAND_CALLBACK] received command {}".format(ros_string_command));
        current_command = json.loads(ros_string_command.data);
        #execute de command
        if ( (self.state is "STOP") and (not 'manual' in current_command)):
            rospy.loginfo("[COMMAND_CALLBACK] going from state {} to WAITING_COMMAND last command {}".format(self.state, current_command['type']));
            self.state = "WAITING_COMMAND"
        elif (current_command['type'] == "FORCE_POI"):
            self.force_poi_command(current_command);
        elif ((self.state is "STOP") and ('manual' in current_command)):
             rospy.loginfo("[COMMAND_CALLBACK] going from state {} please wait to stop autonomous commands yout command {}".format(self.state, current_command['type']));   
        else:
            if (current_command['type'] == "START_NEXT_POI"):
                self.start_next_poi_command(current_command);
            elif (current_command['type'] == "NEXT_POI"):
                self.next_poi_command(current_command);
            elif (current_command['type'] == "FINISH_EXPERIMENT"):
                self.finish_experiment_command(current_command);
            elif (current_command['type'] == "GOTO_POI"):
                self.goto_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DISTANCE_POI"):
                self.verify_distance_poi_command(current_command);
            elif (current_command['type'] == "MANUAL_CHECK_PEOPLE"):
                self.manual_check_people_command(current_command);
            elif (current_command['type'] == "MANUAL_DO_FIRST_EXPERIMENT_FIRST"):
                self.do_manual_first_experiment_first_command(current_command);
            elif (current_command['type'] == "MANUAL_DO_SECOND_EXPERIMENT"):
                self.do_manual_second_experiment_command(current_command);
            elif (current_command['type'] == "MANUAL_SAY_MANY"):
                self.do_manual_speak_to_many(current_command);
            elif (current_command['type'] == "MANUAL_SPEAK_INITIAL"):
                self.do_manual_speak_inital(current_command);
            elif (current_command['type'] == "STOP"):
                self.stop_command(current_command);
            elif (current_command['type'] == "START_EXPERIMENT"):
                self.start_next_poi_command(current_command);
            else:
                rospy.loginfo("[COMMAND_CALLBACK] state {} Wrong Command type command {}".format(self.state, current_command['type']));
 
    def position_subscriber_callback(self, reply):
        #print "[POSITION] {}".format(reply.pose.pose);
        self.current_position = reply.pose.pose;

    #force manual to a poi without autonomous continuity
    def force_poi_command(self, command):
        rospy.loginfo("[FORCE_POI] going from state {} to MANUAL_COMPLETE".format(self.state));
        self.state = "MANUAL_COMPLETE"
        self.move_pub.publish(self.poi_location_manager.get_position(command['poi_name']));
        self.rate.sleep();

    # selecting the new room
    def next_poi_command(self, command):
        rospy.loginfo("[NEXT_POI] going from state {} to GETTING_NEXT_POI".format(self.state));
        self.state = "GETTING_NEXT_POI"
        next_command = {}
        next_command['type'] = "START_NEXT_POI";
        self.poi_info_manager.next_poi();
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    #Setting the variables for the new room
    def start_next_poi_command(self, command):
        rospy.loginfo("[START_NEXT_POI] going from state {} to IDLE".format(self.state));
        self.state = "IDLE"
        if 'manual' in command:
            rospy.loginfo("[START_NEXT_POI] manual command poi {}".format(command['poi_name']));
            if(self.poi_info_manager.set_poi(command['poi_name']) == False):
                rospy.loginfo("[START_NEXT_POI] MANUAL WRONG POI: {}".format(command['poi_name']));
        else:
            rospy.loginfo("[START_NEXT_POI] autonomus command");
        #setting the pois and their position
        self.poi_name = self.poi_info_manager.get_poi_name();
        self.poi_position = self.poi_location_manager.get_position(self.poi_name);
        rospy.loginfo("[START_NEXT_POI] Room: {}".format(self.poi_name));
        
        # make the screen white
        web_msg = pal_web_msgs.msg.WebGoTo();
        web_msg.type = 3;
        web_msg.value = "/static/webapps/client/experiment/white.html";
        self.web_pub.publish(web_msg);
        self.rate.sleep();

        if("ENSTA" in self.poi_name):
            #self.sound_manager.play_ensta();
            rospy.loginfo("ENSTA");

        # GIVE THE cOMMAND FOR GOING
        next_command = {};
        next_command['type'] = "GOTO_POI";
        rospy.loginfo("[START_NEXT_POI] publish next command");
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    #experiment finished
    def finish_experiment_command(self, command):
        rospy.loginfo("[FINISH_EXPERIMENT] BIG EXPERIMENT FINISHED ALL DOORS VISITED OR TRIED");
        rospy.loginfo("[FINISH_EXPERIMENT] going from state {} to EXPERIMENT_FINISHED".format(self.state));
        self.state = "EXPERIMENT_FINISHED";



    def goto_poi_command(self, command):
        rospy.loginfo("[GOTO_POI] going from state {} to GOING_TO_POI".format(self.state));
        self.state = "GOING_TO_POI"
        #set gooal for the new position
        self.move_pub.publish(self.poi_position);
        self.rate.sleep();
        next_command = {};
        next_command['type'] = "VERIFY_DISTANCE_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
        
    def verify_distance_poi_command(self, command):
        #rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to GOING_TO_POI".format(self.state));
        current_distance = self.get_distance(self.current_position, self.poi_position);
        if ( current_distance > self.DISTANCE_ERROR):
            rospy.loginfo("[VERIFY_DISTANCE_POI] distance from point {} ".format(current_distance));
            if( (self.last_point != None) and (self.last_point == self.current_position) ):
                self.tries =  self.tries + 1;
                rospy.loginfo("[VERIFY_DISTANCE_POI] tries {} ".format(self.tries));
                if(self.tries == 3):
                    self.tries = 0;
                    speed = random.randint(0,1);
                    message_type = random.randint(0,1);
                    self.sound_manager.play_let_me_pass(message_type, speed);
                    rospy.sleep(2);
                    self.sound_manager.play_thank_you(speed);
                    
                rospy.sleep(1);
                self.clear_costmaps();
                rospy.sleep(1);
                self.move_pub.publish(self.poi_position);
                self.rate.sleep();
            else:
                rospy.loginfo("[VERIFY_DISTANCE_POI] reset tries {} ".format(self.tries));
                self.tries = 0;
            next_command = {}
            next_command['type'] = "VERIFY_DISTANCE_POI";
            self.last_point = self.current_position;
            rospy.sleep(2);
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        else:
            rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to ARRIVED_POI".format(self.state));
            self.state = "ARRIVED_POI"
            
            rospy.loginfo("[VERIFY_DISTANCE_POI] ARRIVED POI- WAITING CHECK PEOPLE");


            rospy.loginfo("[MANUAL_CHECK_PEOPLE] going from state {} to CHECKING_PEOPLE".format(self.state));
            self.state = "CHECKING_PEOPLE"

            next_command = {}
            next_command['type'] = "AUTONOMOUS_EXPERIMENT";
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
    
    def autonomous_do_experiment(self, command):
        rospy.loginfo("[AUTONOMOUS_EXPERIMENT] going from state {} to EXPERIMENT".format(self.state));
        self.state = "EXPERIMENT";
        if("ENSTA" in self.poi_name):
            rospy.loginfo("[MANUAL SPEAK INITIAL] ARRIVED AT ENSTA");
        else:
            #start looking after one person
            self.look_after_pub.publish("START");
            #wait for him to be centred by our robot
            while True:
                response = rospy.wait_for_message(
                    '/bibpoli/lookup_rep',
                    std_msgs.msg.String);
                rospy.loginfo("[AUTONOMOUS_EXPERIMENT] getting from /bibpoli/lookup_rep:\n {}".format(response));
                if (response.data == "CENTER"):
                    break;
            #say the message
            speed = random.randint(0,1);
            message_type = random.randint(0,1);
            self.sound_manager.play_introduction(message_type, speed);
            rospy.sleep(1);
            message_type = random.randint(0,2);
            self.sound_manager.play_ensta_description(message_type, speed);
            rospy.sleep(1);
            message_type = random.randint(0,1);
            self.sound_manager.play_road_to_ensta(message_type, speed);
            web_msg = pal_web_msgs.msg.WebGoTo();
            web_msg.type = 3;
            web_msg.value = "/static/webapps/client/bibpoli/index.html";
            self.web_pub.publish(web_msg);
            self.rate.sleep();
            rospy.sleep(7);
            self.sound_manager.play_thank_you(speed);
            rospy.sleep(2);
            next_command = {}
            next_command['type'] = "NEXT_POI";
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        #multiple computations
        rospy.sleep(4);

    def manual_do_experiment(self, command):
        rospy.loginfo("[MANUAL_EXPERIMENT] going from state {} to EXPERIMENT".format(self.state));
        self.state = "EXPERIMENT";
        if("ENSTA" in self.poi_name):
            rospy.loginfo("[MANUAL_EXPERIMENT] ARRIVED AT ENSTA");
        else:
            speed = command['speed'];
            message_type = command['introduction_type'];
            self.sound_manager.play_introduction(message_type, speed);
            rospy.sleep(1);
            message_type = command['description_type'];
            self.sound_manager.play_ensta_description(message_type, speed);
            rospy.sleep(1);
            message_type = command['road_type'];
            self.sound_manager.play_road_to_ensta(message_type, speed);
            web_msg = pal_web_msgs.msg.WebGoTo();
            web_msg.type = 3;
            web_msg.value = "/static/webapps/client/bibpoli/index.html";
            self.web_pub.publish(web_msg);
            self.rate.sleep();
            rospy.sleep(7);
            self.sound_manager.play_thank_you(speed);
            rospy.sleep(2);
        #multiple computations
        rospy.sleep(4);

    def stop_command(self, command):
        rospy.loginfo("[STOP] going from state {} to STOP".format(self.state));
        self.state = "STOP";

    def get_distance(self, point1, point2):
        x1 = point1.position.x;
        y1 = point1.position.y;
        x2 = point2.pose.position.x;
        y2 = point2.pose.position.y;
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));


if __name__ == '__main__':
    rospy.init_node('bib_poli_node', anonymous=True);
    infofilename = rospy.get_param('~infofilename', '/home/pal/default_rooms.json')
    try:
        rospy.loginfo("[EXPERIMENT_NODE] STARTED");
        my_logic_manager = ExperimentLogicManager(infofilename);
        rospy.spin();
    except KeyboardInterrupt:
        pass;