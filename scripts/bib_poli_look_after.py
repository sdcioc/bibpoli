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


# Clasa ce se ocupa de logica modului de a hotari pozitia in care
# robotul va avea in fata cele mai multe persoane
class LookUpLogicManager:
	#constructor
    def __init__(self):
        self.rate = rospy.Rate(10);
        # un topic unde se va fisa raspunsul cu numarul de persoane gasite
        self.rep_pub = rospy.Publisher(
                    '/bibpoli/lookup/rep',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        # un bridge pentru transforamrea imagini ros in imagine opencv si viceversa
        self.cvBridge = cv_bridge.CvBridge();
        # detectorul de fete
        self.faceDectector = dlib.get_frontal_face_detector();
        # starea sistemului
        self.state = "STOP";
        # un subscriber pentru comenzile venite de la sistemul central
        rospy.Subscriber("bibpoli/lookup/cmd", std_msgs.msg.String, self.command_subscriber);
        # un Action Client pentru Action Server-ul move_base care primeste ca goal destinatii
        # la care robotul trebuie sa ajunga
        self.move_base_actionlib_client = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
        self.move_base_actionlib_client.wait_for_server();

    #callback-ul pentru comenzile venite
    def command_subscriber(self, command):
        self.state = command.data;
        # daca am primit o comanda de start
        if(self.state == "START"):
            #preiau pozitia curenta
            reply = None;
            while True:
                try:
                    reply = rospy.wait_for_message(
                    '/amcl_pose',
                    geometry_msgs.msg.PoseWithCovarianceStamped, 3)
                    break;
                except rospy.exceptions.ROSException:
                    rospy.loginfo("[ERROR] No Info from amcl_pose tring again");
            #setez pozitia initiala dar fara orientari deocamadata
            initial_pose = reply.pose.pose;
            initial_pose.orientation.x = 0;
            initial_pose.orientation.y = 0;
            initial_pose.position.z = 0;
            # variabile pentru cautarea pozitiei unde s-a gasit numarul maxim de persoane
            max_people = -1;
            max_index = -1;
            rospy.loginfo(" Strart to move trhough indeces");
            # deoarece camera are un unghi de 60 de grade voi roti robotul cu cate
            # 60 de grade ce in radiani inseamna pi/3. Deoarece 360/60 = 6 inseamna ca
            # am doar 6 unchiuri ca sa fac un cerc complet cu robotul
            # incep de la unghiul zero si creez cate un mesaj pentru Action Server.
            # robotul doar se va roti deci pozitia lui va fi aceeasi numai orientarea
            # se va schimb de la pas la pas
            for i in range(0,6):
                # calcularea unghiului teta
                teta = i * math.pi / 3.0;
                # crearea headerul si modificarea
                h = std_msgs.msg.Header();
                h.stamp = rospy.Time.now();
                h.frame_id = "/map";
                # setez orientarile pentru noul unghi
                initial_pose.orientation.z = math.sin(teta/2);
                initial_pose.orientation.w = math.cos(teta/2);
                # creez mesajul pentru server cu headerul si cu positia dorita
                move_base_goal = move_base_msgs.msg.MoveBaseGoal();
                move_base_goal.target_pose.header = h;
                move_base_goal.target_pose.pose = initial_pose;
                rospy.loginfo(" Strart to move to {}".format(i*60));
                # trimit comanda catre server si astept raspunsul timp de 5 secunde maxim
                self.move_base_actionlib_client.send_goal(move_base_goal);
                self.move_base_actionlib_client.wait_for_result(rospy.Duration(5));
                # Fac trei poze si retin numarul maxim de persoane din cele 3 poze
                # in local max. Este posibil ca Detectorul sa nu detecteze mereu
                # fetele din aceasta cazua efectuez mai multe incercari
                local_max = 0;
                j = 0;
                rospy.sleep(2);
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
            # Am obitinut unghiul cel mai bun asa ca ma voi roti la acela dand un mesaj
            # catre Action Server
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
            self.move_base_actionlib_client.wait_for_result(rospy.Duration(8));
            # dupa raspund sistemului central ca m-am pozitionat ca acesta sa poata continua
            # experimentul
            self.rep_pub.publish(json.dumps({'state':'CENTER', 'people':max_people}));
            self.rate.sleep();
            self.state = "STOP";

if __name__ == '__main__':
    rospy.init_node('bibpoli_lookafter_node', anonymous=True);
    try:
        rospy.loginfo("[BIBPOLI_LOOK] STARTED");
        my_logic_manager = LookUpLogicManager();
        rospy.spin();
    except KeyboardInterrupt:
        pass;