#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018
import json
import rospy
import std_msgs.msg
import random

# Clasa ce transmite comenzile manuale
# catre sitemul central
class ExperimentManualCommandManager:
    def __init__(self):
        # topicul pentru comenzile catre sistemul central
        self.command_pub = rospy.Publisher(
                    'bibpoli/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        #rata de publicare
        self.rate = rospy.Rate(10);

    # trimiterea unel comenzi de stio
    def send_stop_command(self):
        next_command = {};
        next_command['type'] = "STOP";
        next_command['manual'] = True;
        print "[INFO][MANUAL_COMMAND] sending STOP";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
  
    # trimiterea unei comenzi ce ii spune sitemului central de la
    # punct de interes sa continuie calea experimentului
    # trebuie precizat in comanda numele punctului de inters
    def send_start_next_poi_command(self, poi_name):
        next_command = {};
        next_command['type'] = "START_NEXT_POI";
        next_command['manual'] = True;
        next_command['poi_name'] = poi_name;
        print "[INFO][MANUAL_COMMAND] sending START_NEXT_POI with poi: {}".format(poi_name);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    # fortarea robotului de a merge la un anumit punct de inters fara
    # a face altceva (precum continuarea experiemntului)
    # atentie aceasta comanda nu verifica daca s-a ajuns la acel punct de interes
    def send_force_poi_command(self, poi_name):
        next_command = {};
        next_command['type'] = "FORCE_POI";
        next_command['manual'] = True;
        next_command['poi_name'] = poi_name;
        print "[INFO][MANUAL_COMMAND] sending FORCE_POI with poi: {}".format(poi_name);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();   

    # trimiterea unei comenzi generale care nu necesita informatii in plus
    def send_general_command(self, command_type):
        next_command = {};
        next_command['type'] = command_type;
        next_command['manual'] = True;
        print "[INFO][MANUAL_COMMAND] sending {}".format(command_type);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    # comanda pentru pornirea sistemului central si experimentului mare
    def send_start_experiment_command(self):
        next_command = {};
        next_command['type'] = "START_EXPERIMENT";
        print "[INFO][MANUAL_COMMAND] sending START_EXPERIMENT";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();       

    #comanda pentru pornirea unui experiment manual
    #trebuie sa contina viteza de vorbire, tipulde introducere, tipul de descriere a ENSTA
    # tipul de descriere a drumului catre ensta
    def send_manual_experiment_command(self, speed, introduction_type, description_type, road_type):
        next_command = {};
        next_command['type'] = "MANUAL_EXPERIMENT";
        next_command['introduction_type'] = introduction_type;
        next_command['description_type'] = description_type;
        next_command['road_type'] = road_type;
        print """[INFO][MANUAL_COMMAND] sending MANUAL_EXPERIMENT with speed: {}; 
        introduction_type: {}; description_type : {};  road_type : {}""".format(speed, introduction_type, description_type, road_type);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

if __name__ == '__main__':
    # initializare nod ros
    rospy.init_node('bib_poli_manual_interface', anonymous=True);
    try:
        # initializarea clasei
        emc =  ExperimentManualCommandManager();
        # Rularea unei bucle ce steapta comenzi ce vor fi transmise
        # catre sistemul central. Daca sistemul este in modul autonom
        # trebuie data o comanda de stop si apoi poate fi data abia
        # o comanda manuala (dupa un timp de 2 secunde)
        # Toate comenzile sunt definite in sitemul central mai putin
        # MANUAL_EXPERIMENT_RANDOM care selecteaza random conditile
        # pentru experiment fara a face robotul sa treaca la urmatorul
        # punct de interes, adica nu il trece in modul autonom dupa
        # terminarea experimentului la punctul de interes curent
        while(True):
            print """[INFO][MANUAL_COMMAND] command types: \n 
                    0 : START_EXPERIMENT\n
                    1 : STOP;\n
                    2 : START_NEXT_POI\n
                    3 : FORCE_POI\n
                    4 : MANUAL_EXPERIMENT\n
                    5 : MANUAL_EXPERIMENT_RANDOM\n
                    6 : FINISH_EXPERIMENT\n
                    7 : GOTO_POI\n
                    8 : NEXT_POI\n
                    9 : EXIT
                    """
            command_number = int(raw_input("Enter your command:\n"));
            if (command_number == 0):
                emc.send_start_experiment_command();
            elif (command_number == 1):
                emc.send_stop_command();
            elif (command_number == 2):
                poi_name = raw_input("Enter POI name");
                emc.send_start_next_poi_command(poi_name);
            elif (command_number == 3):
                poi_name = raw_input("Enter POI name");
                emc.send_force_poi_command(poi_name);
            elif (command_number == 4):
                speed = int(raw_input("Enter speed:"));
                introduction_type = int(raw_input("Enter introduction_type:"));
                description_type = int(raw_input("Enter description_type:"));
                road_type = int(raw_input("Enter road_type:"));
                emc.send_manual_experiment_command(speed, introduction_type, description_type, road_type);
            elif (command_number == 5):
                speed = random.randint(0,1);
                introduction_type = random.randint(0,1);
                description_type = random.randint(0,2);
                road_type = random.randint(0,1);
                emc.send_manual_experiment_command(speed, introduction_type, description_type, road_type);
            else:
                command_type = "";
                if (command_number == 6):
                    command_type = "FINISH_EXPERIMENT"
                elif (command_number == 7):
                    command_type = "GOTO_POI"
                elif (command_number == 8):
                    command_type = "NEXT_POI"
                elif (command_number == 9):
                    break;
                else:
                    print "[ERORRE][MANUAL_COMMAND] command_number {}".format(command_number);
                emc.send_general_command(command_type);
    except KeyboardInterrupt:
        pass;

