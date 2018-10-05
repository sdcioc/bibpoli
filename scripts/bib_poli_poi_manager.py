#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018

import json
import copy

import rospy

#functie de converire a numelui unui punct de interes in calea lui ca parametru ros
def convert_POIName_RosparamName(poi_name):
	prefix = '/mmap/poi/submap_0/';
	if not poi_name.startswith(prefix):
		poi_name = prefix + poi_name;
	return poi_name;

# clasa ce incarca dintr-un fisier informatiile despre punctele de interes
# si le seteaza ca parametri ros pentru a fi folosite informatiile si de alte
# module (aceste fisier sunt sub format json)
class POILoader:
	#constructor
	def __init__(self, filename):
		#citesc un fisier xml cu datele punctelor de interes din camera
		self.filename = filename;
		with open(self.filename) as fd:
			self.points = json.loads(fd.read());

	def set_POIs(self):
		for point in self.points:
			path = convert_POIName_RosparamName(point['poi_name']);
			#tipul informatiei pentru un punct de interes este de forma
			# subharta pe care se afla, numele punctului de interes, pozitia x, pozitia y, pozitia w -> se poate ajunge la Z
			poi_data = ['submap_0', point['poi_name'], point['x'], point['y'], point['w']];
			rospy.set_param(path, poi_data);

# clasa care salveaza din parametri ros punctele de inters in fisier
# sau creeaza un fisier cu informatii suplimentare despre punctele de inters
# acest fisiere sunt sub format json
class POISaver:
	#constructor
	def __init__(self, filename):
		#citesc un fisier json cu datele punctelor de interes din camera
		# sau o sa scriu in acesta
		self.filename = filename;
		self.points = [];

	#salveaza punctele de interes intr-un fisier json
	def save_POIs(self):
		params = rospy.get_param_names();
		prefix = '/mmap/poi/submap_0/';
		params = filter(lambda x: x.startswith(prefix), params);
		poi_names = [p[len(prefix):] for p in params];
		for poi_name in poi_names:
			path = convert_POIName_RosparamName(poi_name)
			rosparam_data = rospy.get_param(path)
			new_data = {};
			new_data['poi_name'] = poi_name;
			new_data['x'] = rosparam_data[2];
			new_data['y'] = rosparam_data[3];
			new_data['w'] = rosparam_data[4];
			self.points.append(copy.deepcopy(new_data));
		data_to_write = json.dumps(self.points);
		with open(self.filename, 'w') as fd:
			fd.write(data_to_write);

	#salveza informatii suplimentare despre punctele de inters intr-un fisier
	def save_info(self):
		params = rospy.get_param_names();
		prefix = '/mmap/poi/submap_0/';
		params = filter(lambda x: x.startswith(prefix), params);
		poi_names = [p[len(prefix):] for p in params];
		for poi_name in poi_names:
			rospy.loginfo("poi_name {}".format(poi_name))
			new_data = {};
			new_data['poi_name'] = poi_name;
			self.points.append(copy.deepcopy(new_data));
		data_to_write = json.dumps(self.points);
		with open(self.filename, 'w') as fd:
			fd.write(data_to_write);

# testarea functioanrii claselor de mai sus
def test_POI_classes():
    poiSaver = POISaver("/home/pal/poiinfo_saver.json")
    poiSaver.save_POIs();
    poiLoader= POILoader("/home/pal/poiinfo_loader.json");
    poiLoader.set_POIs();

if __name__ == '__main__':
	#initializarea nodului si aflarea modului de operare si a numelui fisierului din
	#parametri ros dati
    rospy.init_node('bib_poli_poi_manager', anonymous=True);
    operation_type = rospy.get_param('~operation_type', 'load')    
    filename = rospy.get_param('~filename', '/home/pal/default_pois.json')
    print "[INFO][POI_MANAGER] operation type : {} ; filename : {}".format(operation_type, filename);
	#daca incarca dintr-un fisier punctele de inters in parametrii ros 
    if (operation_type == "load"):
        poiLoader= POILoader(filename);
        poiLoader.set_POIs();
	#daca salveaza intr-un fisier punctele de interes sub forma parametrii ros
    elif (operation_type == "save"):
        poiSaver = POISaver(filename);
        poiSaver.save_POIs();
	#daca salveaza intr-un fisier informatii suplimentare despre punctele de interes sub forma
	# de parametrii ros
    elif (operation_type == "info"):
        poiSaver = POISaver(filename);
        poiSaver.save_info();
    else:
        print "[ERROR][POI_MANAGER] wrong operation type please use load or save";
