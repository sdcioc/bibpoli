#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018

#librari specifice python
import math
import json
# pentru a putea face deep copy la un obiect cand il adaugam
# intr-o lista
import copy
# random pentru alegerea conditilor
import random
# supprocess pentru a apela comenzi din linia de comanda
# precum aplay pentru redarea fisierelor wav
import subprocess

# librari specifice ros
import rospy
# librarie pentru pachetele ros folosite pentru a gasi
# pozitia unde se afla pachetele noastre
import rospkg

#mesaje ros folosite si mesaje de sreviciu
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import pal_web_msgs.msg
import std_srvs.srv

# convertirea datelor unui punct de interes intr-o pozitie de harta
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
#convertire din pozitie de harta in date pentru punct de interes
def convert_MapPosition_POIPosition(position):
    x = position.position.x;
    y = position.position.y;
    w = 2 * math.acos(position.orientation.w);
    return (x, y, w);

#gasriea path-ului catre parametrul ros unde se afla un anumit punct de interes
def convert_POIName_RosparamName(poi_name):
    prefix = '/mmap/poi/submap_0/';
    if not poi_name.startswith(prefix):
        poi_name = prefix + poi_name;
    return poi_name;

# Clasa care optine pozitia pe harta a unui punct de interes
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
#si le parcurge
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

# clasa ce se ocupa de redarea sunetelor
class SoundManager:
    def __init__(self):
        #utilitarul de redare
        self.utility = "aplay ";
        #setarea volumului
        self.general_volume_param = "/pal/general_volume";
        self.playback_volume_param = "/pal/playback_volume";
        rospy.set_param(self.general_volume_param, 76);
        rospy.set_param(self.playback_volume_param, 76);
        #calea catre pachet si catre directorul cu fisierele audio
        rospack = rospkg.RosPack();
        self.prefix = rospack.get_path('bib_poli_package') + "/config/sounds/";
    
    #redarea mesajul de introducere
    def play_introduction(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "1";
        else:
            command = command + "2";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + ".wav";
        subprocess.check_output(command.split());

    #redarea mesajului de descriere a facultatii
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
            command = command + ".wav";
        subprocess.check_output(command.split());
    
    #redarea mesajului pentru descrierea cai de urmatul
    #pentru a ajunge la standul ENSTA
    def play_road_to_ensta(self, message_type, speed):
        command = self.utility + self.prefix;
        if(message_type == 0):
            command = command + "6";
        else:
            command = command + "7";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + ".wav";
        subprocess.check_output(command.split());
    
    #redarea mesajului de multumire
    def play_thank_you(self, speed):
        command = self.utility + self.prefix;
        command = command + "11";
        if(speed == 1):
            command = command + "m.wav";
        else:
            command = command + ".wav";
        subprocess.check_output(command.split());

    #redarea unui mesaj prin care se cere permisiunea
    #de a se elibera calea robotului
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
            command = command + ".wav";
        subprocess.check_output(command.split());
    
# Clasa care decide toata logica experiemntului si mdoului de functionare a robotului
# 
#
class LogicManager:
	#constructor
    def __init__(self, infofilename):
        # clasa pentru informatii legate de punctele de interes
        self.poi_info_manager = POIInfoManager(infofilename);
        # clasa pentru locatia punctelor de interes
        self.poi_location_manager = POILocationManager();
        # clasa pentru redarea mesajelor audio
        self.sound_manager = SoundManager();
        # starea sistemul
        self.state = "INITIAL";
        # marsa de eroare fata de distanta de punctul de interes
        self.DISTANCE_ERROR = 0.6;
        # calea catre parametrii ce reprezinta distanta de luat fata de obstacole
        # pentru sonar si laser
        self.sonar_param = "/speed_limit/limitess/base_sonars/obstacle_max/dist";
        self.laser_param = "/speed_limit/limitess/base_laser/obstacle_max/dist";
        # distanta normala fata de obstacole
        self.obstacle_normal_dist = 0.3;
        # distanta in cazul usiilor
        self.obstacle_door_dist = 0.05;
        # numarul de oamnei
        self.people_number = 0;
        # rate de publicare pe topicuri
        self.rate = rospy.Rate(10);
        # numarul de incercari de efectuate de a reveni la traseul curent
        self.tries = 0;
        # numele punctului de interes curent si pozitia acestuia pe harta
        self.poi_name = None;
        self.poi_position = None;
        # ultima pozitie raportata de robot
        self.last_point = None;
        # contorul pentru numarul experiemntului
        self.contor = 0;
        # vectorul cu evenimente
        self.events = [];
        # calea catre directorul de loguri
        rospack = rospkg.RosPack();
        self.path_prefix = rospack.get_path('bib_poli_package') + "/logs/";
        #calea catre fisierul cu evenimete TODO: sa o fac modificabila
        self.eventsFile = "/home/pal/hddextern/bibpoli/" + "events.json"
        # topicul pentru comenzile luate de sistemul central
        self.command_pub = rospy.Publisher(
                    '/bibpoli/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        # topicul pentru setarea unei destinatii a robotului
        self.move_pub = rospy.Publisher(
                    '/move_base_simple/goal',
                        geometry_msgs.msg.PoseStamped,
                        latch=True, queue_size=5);
        # topicul pentru comenzile pentru interfata web
        self.web_cmd_pub = rospy.Publisher(
                    '/bibpoli/web/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        # topicul pentru comenzile pentru urmarirea persoanelor
        self.look_after_pub = rospy.Publisher(
                    '/bibpoli/lookup/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        # topicul pentru comenzile pentru inregistrarea rosbagurilor
        self.rosbag_pub = rospy.Publisher(
                    '/bibpoli/rosbag/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        # asteptare pentru inregistrarea topicurilor
        rospy.sleep(3);
        # subscriber pentru comenzile catre sistemul central
        rospy.Subscriber("bibpoli/cmd", std_msgs.msg.String, self.command_subscriber_callback);
        # subscriber pentru pozitia robotului
        rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.position_subscriber_callback);
        # asteptare pentru inregistrarea acestora
        rospy.sleep(3);
        # asteptarea pentru pornirea serviciul de eliberare a costmapurilor
        rospy.wait_for_service('move_base/clear_costmaps');
        # functia pentru apelarea serviciului respectiv
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty);

    # calbback pentru comenzile date catre sistemul central
    def command_subscriber_callback(self, ros_string_command):
        rospy.loginfo("[COMMAND_CALLBACK] received command {}".format(ros_string_command));
        # decodarea comenzi (aceasta este transmisa sub forma de json in format String)
        current_command = json.loads(ros_string_command.data);
        # daca sistemul este in starea stop asteapta urmatorea comanda automata pentru
        # a opri sistemul din starea autonoma
        if ( (self.state is "STOP") and (not 'manual' in current_command)):
            rospy.loginfo("[COMMAND_CALLBACK] going from state {} to WAITING_COMMAND last command {}".format(self.state, current_command['type']));
            self.state = "WAITING_COMMAND"
        # daca este o comanda de a forta robotul de a ajunge intr-un punct de interes
        # caz de uregenta cand acesta nu mai raspunde la comenzi
        elif (current_command['type'] == "FORCE_POI"):
            self.force_poi_command(current_command);
        # daca se primeste o comanda manual inainte de a se iesi din modul autonom
        elif ((self.state is "STOP") and ('manual' in current_command)):
             rospy.loginfo("[COMMAND_CALLBACK] going from state {} please wait to stop autonomous commands yout command {}".format(self.state, current_command['type']));   
        else:
            # executa comanda relevanta tipul comenzii
            if (current_command['type'] == "START_NEXT_POI"):
                self.start_next_poi_command(current_command);
            elif (current_command['type'] == "NEXT_POI"):
                self.next_poi_command(current_command);
            elif (current_command['type'] == "FORCE_POI"):
                self.force_poi_command(current_command);
            elif (current_command['type'] == "FINISH_EXPERIMENT"):
                self.finish_experiment_command(current_command);
            elif (current_command['type'] == "GOTO_POI"):
                self.goto_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DISTANCE_POI"):
                self.verify_distance_poi_command(current_command);
            elif (current_command['type'] == "AUTONOMOUS_EXPERIMENT"):
                self.autonomous_do_experiment(current_command);
            elif (current_command['type'] == "MANUAL_EXPERIMENT"):
                self.manual_do_experiment(current_command);
            elif (current_command['type'] == "STOP"):
                self.stop_command(current_command);
            elif (current_command['type'] == "START_EXPERIMENT"):
                self.start_next_poi_command(current_command);
            else:
                rospy.loginfo("[COMMAND_CALLBACK] state {} Wrong Command type command {}".format(self.state, current_command['type']));
 
    # functia de callback pentru pozitia robotului care actualizeaza pozitia curent a robotului
    # geometry_msgs/PoseStamped
    def position_subscriber_callback(self, reply):
        #print "[POSITION] {}".format(reply.pose.pose);
        self.current_position = reply.pose.pose;

    #force manual to a poi without autonomous continuity
    def force_poi_command(self, command):
        rospy.loginfo("[FORCE_POI] going from state {} to MANUAL_COMPLETE".format(self.state));
        self.state = "MANUAL_COMPLETE"
        self.move_pub.publish(self.poi_location_manager.get_position(command['poi_name']));
        self.rate.sleep();

    # Aflarea urmatorului punct de interes din cadrul sistemului
    def next_poi_command(self, command):
        rospy.loginfo("[NEXT_POI] going from state {} to GETTING_NEXT_POI".format(self.state));
        self.state = "GETTING_NEXT_POI"
        next_command = {}
        next_command['type'] = "START_NEXT_POI";
        self.poi_info_manager.next_poi();
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    # comanda pentru pornirea experiemntului de la un anumit contor
    # si pornirea de la primul punct de interes din lista
    def start_experiment(self, command):
        self.contor = command['contor'];
        next_command = {};
        next_command['type'] = "START_NEXT_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    # setarea variabilelor pentru urmatorul punct de interes
    def start_next_poi_command(self, command):
        rospy.loginfo("[START_NEXT_POI] going from state {} to IDLE".format(self.state));
        self.state = "IDLE"
        #daca am avut un experiemnt inainte si evenimente de la el le scriem in
        #fisierul de evenimente si eliberam vectorul de fisiere
        if (len(self.events) > 0):
            data_to_write = json.dumps(self.events);
            with open(self.eventsFile, 'a') as fd:
                fd.write(data_to_write);
                self.events = [];
        #este un nou experiment deci marim contorul
        self.contor = self.contor + 1;
        # daca este o comanda manual clasa
        # ce se ocupa de informatii pentru punctele de interes
        # la pozitia acestuia astfel continuam cu punct oferit de acesta inainte
        # in comanda next_poi (cazul autonom)
        if 'manual' in command:
            rospy.loginfo("[START_NEXT_POI] manual command poi {}".format(command['poi_name']));
            if(self.poi_info_manager.set_poi(command['poi_name']) == False):
                rospy.loginfo("[START_NEXT_POI] MANUAL WRONG POI: {}".format(command['poi_name']));
        else:
            rospy.loginfo("[START_NEXT_POI] autonomus command");
        # setam numele si pozitia noului punct de interes
        self.poi_name = self.poi_info_manager.get_poi_name();
        self.poi_position = self.poi_location_manager.get_position(self.poi_name);
        rospy.loginfo("[START_NEXT_POI] Room: {}".format(self.poi_name));
        
        # setam interfata web la logo-ul ENSTA
        self.web_cmd_pub.publish("SHOW_LOGO");
        self.rate.sleep();

        # Daca punctul la care vom merge este chiar ensta
        if("ensta" in self.poi_name):
            #self.sound_manager.play_ensta();
            rospy.loginfo("ENSTA");

        # setam urmatoarea comanda pentru deplasearea autonoma catre acest punct
        next_command = {};
        next_command['type'] = "GOTO_POI";
        rospy.loginfo("[START_NEXT_POI] publish next command");
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    # terminarea experimentului (in cazul nostru se ajunge manual pentru ca nu trebuie
    # sa terminam experimentul cand au fost vizitate toate punctele de interes)
    def finish_experiment_command(self, command):
        rospy.loginfo("[FINISH_EXPERIMENT] going from state {} to EXPERIMENT_FINISHED".format(self.state));
        self.state = "EXPERIMENT_FINISHED";


    # comanda pentru deplasearea la noul punct de interes
    def goto_poi_command(self, command):
        rospy.loginfo("[GOTO_POI] going from state {} to GOING_TO_POI".format(self.state));
        self.state = "GOING_TO_POI"
        event = {};
        event['contor'] = self.contor;
        event['type']= "GO_TO_POI";
        event['poi_name'] = self.poi_name;
        event['time'] = rospy.get_time();
        self.events.append(copy.deepcopy(event));
        # setam destinatia pentru deplasarea autonoma
        self.move_pub.publish(self.poi_position);
        self.rate.sleep();
        # transmitem urmatoarea comanda care verifica daca s-a ajuns la destinatie
        next_command = {};
        next_command['type'] = "VERIFY_DISTANCE_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    # comanda care verifica daca s-a ajuns la destinatia data de punctul de interes
    def verify_distance_poi_command(self, command):
        #rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to GOING_TO_POI".format(self.state));
        # se calculeaza distanta de la pozitia robotului la punctul de interes
        current_distance = self.get_distance(self.current_position, self.poi_position);
        # Daca este mai mare ca marja de eroare
        if ( current_distance > self.DISTANCE_ERROR):
            rospy.loginfo("[VERIFY_DISTANCE_POI] distance from point {} ".format(current_distance));
            # daca avem retinuta ultima pozitie a robotului si aceasta este aceeasi ca pozitia
            # curent inseamna ca robotul nostru s-a blocat si incercam sa golim costmapuri
            # sa vedem daca putem totusi sa ne deplasam catre acest punct setenadul din nou
            # ca destinatie pentru deplasarea autonoma
            if( (self.last_point != None) and (self.last_point == self.current_position) ):
                self.tries =  self.tries + 1;
                rospy.loginfo("[VERIFY_DISTANCE_POI] tries {} ".format(self.tries));
                # Daca robotul este la a treia incercare inseamna ca acesta este blocat
                # de o persoana asa ca in plus redam un mesaj audio care solicita sa ii
                # fie eliberata calea asteapta o perioada dupa care multumeste
                if(self.tries == 3):
                    event = {};
                    event['pose'] = {};
                    event['pose']['position']['x'] = self.current_position.position.x;
                    event['pose']['position']['y'] = self.current_position.position.y;
                    event['pose']['position']['z'] = self.current_position.position.z;
                    event['pose']['orientation']['x'] = self.current_position.orientation.x;
                    event['pose']['orientation']['y'] = self.current_position.orientation.y;
                    event['pose']['orientation']['z'] = self.current_position.orientation.z;
                    event['pose']['orientation']['w'] = self.current_position.orientation.w;
                    event['contor'] = self.contor;
                    event['type']= "BLOCKED";
                    event['poi_name'] = self.poi_name;
                    event['time'] = rospy.get_time();
                    self.events.append(copy.deepcopy(event));
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
            # deoarece nu a ajuns la punctul de interes urmatoarea comanda va fi tot
            # pentru verificarea daca s-a ajuns la acest punct de interes
            next_command = {}
            next_command['type'] = "VERIFY_DISTANCE_POI";
            self.last_point = self.current_position;
            rospy.sleep(2);
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        else:
            # in caz de s-a ajuns la punctul de interes urmatoarea comanda este 
            # a porni experimentul autonom care incepe prin cautarea persoanelor
            rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to ARRIVED_POI".format(self.state));
            self.state = "ARRIVED_POI"
            event = {};
            event['contor'] = self.contor;
            event['type']= "REACHED_POI";
            event['poi_name'] = self.poi_name;
            event['time'] = rospy.get_time();
            self.events.append(copy.deepcopy(event));
            
            rospy.loginfo("[VERIFY_DISTANCE_POI] ARRIVED POI- WAITING CHECK PEOPLE");


            rospy.loginfo("[MANUAL_CHECK_PEOPLE] going from state {} to CHECKING_PEOPLE".format(self.state));
            self.state = "CHECKING_PEOPLE"

            next_command = {}
            next_command['type'] = "AUTONOMOUS_EXPERIMENT";
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
    
    # comanda ce executa un experiment autonom
    def autonomous_do_experiment(self, command):
        rospy.loginfo("[AUTONOMOUS_EXPERIMENT] going from state {} to EXPERIMENT".format(self.state));
        self.state = "EXPERIMENT";
        # Daca s-a ajuns la standul ENSTA
        if("ENSTA" in self.poi_name):
            rospy.loginfo("[MANUAL SPEAK INITIAL] ARRIVED AT ENSTA");
        else:
            # trimitem comanda pentru modul de cautare a personelor
            self.look_after_pub.publish("START");
            # Asteptam ca acest modul sa centralizeze o persoana
            while True:
                response = rospy.wait_for_message(
                    '/bibpoli/lookup/rep',
                    std_msgs.msg.String);
                rospy.loginfo("[AUTONOMOUS_EXPERIMENT] getting from /bibpoli/lookup_rep:\n {}".format(response));
                response_dict = json.loads(response.data);
                if (response_dict['state'] == "CENTER"):
                    event = {};
                    event['people'] = response_dict['people'];
                    event['contor'] = self.contor;
                    event['type']= "FIND_PEOPLE";
                    event['poi_name'] = self.poi_name;
                    event['time'] = rospy.get_time();
                    self.events.append(copy.deepcopy(event));
                    break;
            # Incepem ceea ce trebuie sa spunem sigur selectand random tipul de mesaje folosite
            # din fiecare grupa si viteza de vorbire a robotului
            # se efectueaza experimentul
            # dupa se da comanda pentru trecerea la urmatorul punct de interes
            self.experiment(random.randint(0,1), random.randint(0,1), random.randint(0,2), random.randint(0,1))
            next_command = {}
            next_command['type'] = "NEXT_POI";
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        #multiple computations
        rospy.sleep(4);

    #comanda pentru efectuarea unui experiemnt manual
    def manual_do_experiment(self, command):
        rospy.loginfo("[MANUAL_EXPERIMENT] going from state {} to EXPERIMENT".format(self.state));
        self.state = "EXPERIMENT";
        # Daca s-a ajuns la standul ENSTA
        if("ENSTA" in self.poi_name):
            rospy.loginfo("[MANUAL_EXPERIMENT] ARRIVED AT ENSTA");
        else:
            # viteza robotului si tipul de mesaje folosite sunt in cadrul comenzii
            # se efectuaza experimentul cu aceste detalii
            # dupa care se va astepta o noua comanda manuala
            self.experiment(command['speed'], command['introduction_type'], command['description_type'], command['road_type'])
        #multiple computations
        rospy.sleep(4);
    
    def experiment(self, speed, introduction_type, description_type, road_type):
        # se incepe cu mesajul de introducere, dupa mesajul de descriere pentru ensta
        # mesajul pentru ruta catre ensta si afisarea hartii cu calea catre standul ENSTA
        # pe interfata robotului si spunerea mesajului de multumire prin care il roaga
        # sa ia parte la un chestionar

        #se activeaza inregistrarea rosbagului
        rosbag_command = {};
        rosbag_command['state'] = "START";
        rosbag_command['contor'] = self.contor;
        self.rosbag_pub.publish(json.dumps(rosbag_command));
        event = {};
        event['speed'] = speed;
        event['introduction_type'] = introduction_type;
        event['description_type'] = description_type;
        event['road_type'] = road_type;
        event['contor'] = self.contor;
        event['type']= "INTERACTION_STARTED";
        event['poi_name'] = self.poi_name;
        event['time'] = rospy.get_time();
        self.events.append(copy.deepcopy(event));
        self.sound_manager.play_introduction(introduction_type, speed);
        rospy.sleep(1);
        self.sound_manager.play_ensta_description(description_type, speed);
        rospy.sleep(1);
        event = {};
        event['contor'] = self.contor;
        event['type']= "SHOW_MAP";
        event['poi_name'] = self.poi_name;
        event['time'] = rospy.get_time();
        self.events.append(copy.deepcopy(event));
        self.web_cmd_pub.publish("SHOW_MAP");
        self.rate.sleep();
        self.sound_manager.play_road_to_ensta(road_type, speed);
        rospy.sleep(7);
        self.sound_manager.play_thank_you(speed);
        # chestionar
        # se afiseaza chestionarul dupa care se asteapta
        # pornirea si terminarea chestioanrului de catre participant
        event = {};
        event['contor'] = self.contor;
        event['type']= "START_QUESTIONS";
        event['poi_name'] = self.poi_name;
        event['time'] = rospy.get_time();
        self.events.append(copy.deepcopy(event));
        self.web_cmd_pub.publish("START_EXPERIMENT");
        try:
            while True:
                reply = rospy.wait_for_message(
                '/bibpoli/web/rep',
                std_msgs.msg.String, 30);
                rospy.loginfo("[EXPERIMENT]Start gettin {}".format(reply));
                if (reply.data == "pressStart"):
                    break;
            while True:
                finish = rospy.wait_for_message(
                '/bibpoli/web/rep',
                std_msgs.msg.String, 700);
                rospy.loginfo("[EXPERIMENT]Quit gettin {}".format(finish));
                if (finish.data == "pressQuit"):
                    break;
        except:
            rospy.loginfo("[EXPERIMENT] experiment nepornit sau neterminat");
        event = {};
        event['contor'] = self.contor;
        event['type']= "INTERACTION_FINISHED";
        event['poi_name'] = self.poi_name;
        event['time'] = rospy.get_time();
        self.events.append(copy.deepcopy(event));
        # se opreste inregistrarea rosbagului
        rosbag_command = {};
        rosbag_command['state'] = "STOP";
        rosbag_command['contor'] = self.contor;
        self.rosbag_pub.publish(json.dumps(rosbag_command));

    # comanda de a merge in starea stop
    def stop_command(self, command):
        rospy.loginfo("[STOP] going from state {} to STOP".format(self.state));
        self.state = "STOP";

    #calcularea distante intre 2 puncte de pe harta
    #primul punct este de tip geometry_msgs/Pose
    #al doilea punct este de tip geometry/PoseStamped
    def get_distance(self, point1, point2):
        x1 = point1.position.x;
        y1 = point1.position.y;
        x2 = point2.pose.position.x;
        y2 = point2.pose.position.y;
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));


if __name__ == '__main__':
    # declarea nodului preluarea parametrului ce ofera detalii despre punctele de inters
    rospy.init_node('bib_poli_node', anonymous=True);
    infofilename = rospy.get_param('~infofilename', '/home/pal/default_info.json')
    try:
        # se executa sistemul central pana la oprirea acestuia prin CTRL+C
        # inainte de efectuarea acestui procedeu aveti grija ca sistemul central
        # sa fie in starea de terminare a experimentului
        rospy.loginfo("[EXPERIMENT_NODE] STARTED");
        my_logic_manager = LogicManager(infofilename);
        rospy.spin();
    except KeyboardInterrupt:
        pass;