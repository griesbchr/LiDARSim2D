The program can be called using one of two ways:
	1) via the main.py function
		->select simulation parameters in head section of main.py
		  OR
		->select simulation parameters "model", "scenario" and "T_SIM" via commandline using the following format:
			main.py "model" "scenatio" "T_SIM" 	eg.: main.py realsensor scenario_crossing 5 (each argument is passed as a string)
			commandline parameters are optional, if parameters are left out the the parameters given by the script are used.
		-> execution time and other simulation parameters will be loged in the Logs.txt file
		-> visual representation of the results can be plotted by setting the "viewer" flag to 1 in the file head
		-> profiling of the getPointcloud function can be plotted by setting the "profiling" flag to 1 in the file head
		-> Pointclouds are calculated using the getPointCloudRealSensor_rt and getPointCloudModelX_rt functions
		-> used for benchmarking and testing of the functionality of the program (RunBenchmarks.py)

	2) via the SceneViewer.py function
		-> calls the GUI which further calls the program
		-> parameters can selected within the GUI
		-> no logging of the execution times because execution time is limited by the plotting speed of the plotting framework
		-> Pointclouds for models are calculated using the getPointCloudModelX_rt functions
		-> Pointcloud for the realsensor is calculated using the getPointCloudRealSensor (NOT the getPointCloudRealSensor_rt) function
		-> provides option to directly compare the PC (pointcloud) of the realsensor calculation option with the lidar model calculation options
		-> used to provide visual demonstration examples of a scenatio
		-> timeslider can be controlled using the arrow keys (also in combination with the "alt" key) to navigate in scenarios

The lidar parameters can be tuned in the data.py file in the egocar class.

Explaination of files:
The project can be devided into 3 categories:
	1) Framework that provides data about the scenario to the getPointcloud functions:
		-> data.py: contails all the classes and class methods
		-> scenarios.py: scenarios (instances of scenario class) are initialized here eg. scenario_crossing, scenario_overtake

		-> viewer.py: plots scenario and pointcloud (used if viewer flag is activated in main.py file)

	2) getPointcloud functions and its subfuctions:
		-> getPointcloud.py: contains getPointCloud() functions
		-> coordTrans.py: subfunction of getPointCloud(), contains all kind of coordinate transformations
		-> getRayIntersections.py: subfunction of getPointCloud()

	3) files that call the program:
		-> main.py
		-> sceneviwer.py
	
Additionally a file with a pseudocode function to extract the arguments for the PC calculation exists.
	-> getPCargsFromStream.py
	-> suggests which argument can be extracted from which attribute in the stream

coordinate systems: 
	->global coordinate system: Ground Truth coordinate system, used to describe Scenarios and all its attributes
	->vehicle coordinate system: Origin is at the center of the rear axis of a vehicle, ego_x_y_th/target_x_y_th describe translation and rotation of vehicle coordinate system vs the global coordinate system, used to describe car vertices and lidar mounting position (relative to the car). 
	->sensor (or hit) coordinate system: origin is the position of the lidar, rotation is always 0 (that means no rotation relative to the global coordinate system), getRayIntersection() function returns hit coordinates in this coordinate system. 

numba: 
numba can be "deactivated" by commenting out the "@njit(....)" headers of the functions. 









