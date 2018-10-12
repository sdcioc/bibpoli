#! /usr/bin/python
import rospy
import std_msgs.msg
import subprocess

## face detection libraries
import json
import math
import time

# clasa ce se ocupa de logica de inregistrare a rosbackurilor
class RosbagManager:
	#constructor
    def __init__(self):
        self.rate = rospy.Rate(10);
        # starea modulului
        self.state = "STOP";
        # utilitarul folosit
        self.utility = "rosbag record "
        #conotrul ce reprezinta numarul experimentului
        self.contor = 0;
        # topicurile inregistrate si durata pentru verificare daca se mai face sau nu inregistrarea
        self.topics = " --duration=3m /xtion/depth/image_rect /xtion/rgb/image_rect_color /thermal_image"
        # un subscriber pentru comenzile de la sistemul central
        rospy.Subscriber("bibpoli/rosbag/cmd", std_msgs.msg.String, self.command_subscriber);

    # o functie de loop in care se verifica daca modul este in starea de inregistrare
    # atunci inregistreaza pentru 3 minute dupa care face iarasi aceeasi verificare
    # daca nu este in starea de inregistrare asteapta o secunda pentru comanda
    def loop(self):
        while not rospy.is_shutdown():
            if (self.state == "START"):
                command = self.utility + "-o /home/pal/hddextern/bibpoli/exp_" + str(self.contor) + "_" + self.topics;
                subprocess.check_output(command.split());
            else:
                rospy.sleep(1);

    # calbbackul pentru comenzi (setez dinainte contorul ca sa nu inceapa sa inregistreze pe vechiul contor)
    def command_subscriber(self, command):
        my_dict = json.loads(command.data);
        if((self.contor < int(my_dict['contor'])) and (my_dict['state'] == "START")):
            self.contor = int(my_dict['contor']);
            self.state = my_dict['state'];
        elif((self.contor == int(my_dict['contor'])) and (my_dict['state'] == "STOP")):
            self.state = my_dict['state'];

if __name__ == '__main__':
    rospy.init_node('bibpoli_rosbag_node', anonymous=True);
    try:
        rospy.loginfo("[BIBPOLI_ROSBAG] STARTED");
        my_logic_manager = RosbagManager();
        my_logic_manager.loop();
    except KeyboardInterrupt:
        pass;