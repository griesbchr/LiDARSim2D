import numpy as np

import data
from data import StaticTarget
from data import Scene

def getScenarioNames():
    return ["Scenario_Benchmark", "Scenario1", "Scenario_only_ego", "Scenario_not_implemented", "Scenario_debug", "Scenario_crossing",
            "Scenario_B_S0",
            "Scenario_B_S1",
            "Scenario_B_S2",
            "Scenario_B_S3",
            "Scenario_B_S4",
            "Scenario_B_S5",
            "Scenario_B_S6",
            "Scenario_B_S7",
            "Scenario_B_S8",
            "Scenario_B_S9",

            ] #has to match name of Function without "get"

def getScenario_Benchmark(TSIM, realsensor):
    '''
    A scenario contains stationary and moving objects.
        Stationary objects are defined with the Scene class which contains instances of the StaticTarget class.
        Moving objects are the EgoCar and the TargetCars.
    :return:
        t: time vector for scenario
        scene: stationary objects
        ego_x_y_th: contains x,y position and horizontal angle at each timestep defines in time vector
        target_x_y_th: list that contains for each target vehicle:  x,y position and horizontal angle at each timestep defines in time vector
    '''

    #select Scene, scenes can be created settung up a getStaticScene function below
    scene = getStaticScene4()

    #electing one ego car and various target cars
    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh,-1.5,0), 35)    #((x_init, y_init, theta_init), velocity_init)


    targetcar1 = data.TargetCar()   #Adds car to target to TargetCar.targetlist -> Class variable!!
    targetcar1.init((-8,-1.5,0),20) #((x_init, y_init, theta_init), velocity_init)
    targetcar2 = data.TargetCar()
    targetcar2.init((10,-1.5,0),15)
    targetcar3 = data.TargetCar()
    targetcar3.init((25,1.5,-180),15)
    targetcar4 = data.TargetCar()
    targetcar4.init((37.5,-13,90),15)
    targetcar5 = data.TargetCar()
    targetcar5.init((37.5,-28,90),15)
    targetcar6 = data.TargetCar()
    targetcar6.init((35,1.5,-180),15)
    targetcar7 = data.TargetCar()
    targetcar7.init((5,1.5,-180),15)
    targetcar8 = data.TargetCar()
    targetcar8.init((32.5,26,-90),15)

    #Initializing w-vector that defines movement of egocar
    w_value = 60
    w_value2 = 76.5
    if realsensor:
        idxperturn = egocar._n_rays      #calculating time indices for 1 turn
    else:
        idxperturn = 1

    w_curve = np.full(3*idxperturn, w_value)         #curve left, -w_curve is curve right
    w_curve2 = np.full(3*idxperturn, w_value2)         #curve left, -w_curve is curve right
    w_0 = np.full(3*idxperturn, 0)                  #no curve

    w_lane_change_right = np.stack((-w_curve, w_curve,  w_curve, -w_curve),axis=0).flatten()
    w_lange_change_left = -w_lane_change_right

    w_ego = np.stack((w_lane_change_right,w_lange_change_left),axis=0).flatten()

    w_leftturn = np.stack((w_0, w_0, w_curve2, w_curve2, w_0, w_0, w_0, -w_curve2, -w_curve2))

    #Initialize Target cars w vectors in w_target_list, can be changed the same way as w_ego
    if len(data.TargetCar.targetlist) > 0:
        w_target_list = [None for i in range(len(data.TargetCar.targetlist))] #Initing as None means targets drive straight

        w_target_list[3] = w_leftturn.flatten()
    else:
        w_target_list = None
    scenario = data.Scenario(TSIM, realsensor, scene, egocar, w_ego, data.TargetCar.targetlist, w_target_list)

    return scenario

def getScenario1(TSIM, realsensor):
    '''
    A scenario contains stationary and moving objects.
        Stationary objects are defined with the Scene class which contains instances of the StaticTarget class.
        Moving objects are the EgoCar and the TargetCars.
    :return:
        t: time vector for scenario
        scene: stationary objects
        ego_x_y_th: contains x,y position and horizontal angle at each timestep defines in time vector
        target_x_y_th: list that contains for each target vehicle:  x,y position and horizontal angle at each timestep defines in time vector
    '''

    #select Scene, scenes can be created settung up a getStaticScene function below
    scene = getStaticScene2()

    #electing one ego car and various target cars
    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh,-1.5,0), 35)    #((x_init, y_init, theta_init), velocity_init)


    targetcar1 = data.TargetCar()   #Adds car to target to TargetCar.targetlist -> Class variable!!
    targetcar1.init((-8,-1.5,0),20) #((x_init, y_init, theta_init), velocity_init)
    targetcar2 = data.TargetCar()
    targetcar2.init((10,-1.5,0),15)
    targetcar3 = data.TargetCar()
    targetcar3.init((25,1.5,-180),15)

    #Initializing w-vector that defines movement of egocar
    w_value = 60
    if realsensor:
        idxperturn = egocar._n_rays      #calculating time indices for 1 turn
    else:
        idxperturn = 1

    w_curve = np.full(3*idxperturn, w_value)         #curve left, -w_curve is curve right
    w_0 = np.full(3*idxperturn, 0)                  #no curve

    w_lane_change_right = np.stack((-w_curve, w_curve,  w_curve, -w_curve),axis=0).flatten()
    w_lange_change_left = -w_lane_change_right

    w_ego = np.stack((w_lane_change_right,w_lange_change_left),axis=0).flatten()

    #Initialize Target cars w vectors in w_target_list, can be changed the same way as w_ego
    if len(data.TargetCar.targetlist) > 0:
        w_target_list = [None for i in range(len(data.TargetCar.targetlist))] #Initing as None means targets drive straight


    scenario = data.Scenario(TSIM, realsensor, scene, egocar, w_ego, data.TargetCar.targetlist, w_target_list)

    return scenario

def getScenario_only_ego(TSIM, realsensor):
    scene = getStaticScene2()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh,-1.5,0), 2)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar)

    return scenario

def getScenario_debug(TSIM, realsensor):
    scene = getStaticScene3()

    egocar = data.EgoCar()
    egocar.init((-egocar._L-35 + egocar._foh,-1.5,0), 35)

    #targetcar1 = data.TargetCar()   #Adds car to target to TargetCar.targetlist -> Class variable!!
    #targetcar1.init((-egocar._L + egocar._foh-10,-1.5,0),20) #((x_init, y_init, theta_init), velocity_init)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)


    return scenario


def getScenario_crossing(TSIM, realsensor):
    scene = getStaticScene4()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh,-1.5,0), 15)

    targetcar1 = data.TargetCar()   #Adds car to target to TargetCar.targetlist -> Class variable!!
    targetcar1.init((-egocar._L + egocar._foh+35,20,-90),15) #((x_init, y_init, theta_init), velocity_init)


    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario

def getScenario_B_S0(TSIM, realsensor):
    scene = getStaticSceneB_0()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S1(TSIM, realsensor):
    scene = getStaticSceneB_1()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S2(TSIM, realsensor):
    scene = getStaticSceneB_2()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S3(TSIM, realsensor):
    scene = getStaticSceneB_3()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S4(TSIM, realsensor):
    scene = getStaticSceneB_4()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S5(TSIM, realsensor):
    scene = getStaticSceneB_5()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S6(TSIM, realsensor):
    scene = getStaticSceneB_6()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S7(TSIM, realsensor):
    scene = getStaticSceneB_7()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S8(TSIM, realsensor):
    scene = getStaticSceneB_8()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getScenario_B_S9(TSIM, realsensor):
    scene = getStaticSceneB_9()

    egocar = data.EgoCar()
    egocar.init((-egocar._L + egocar._foh, -1.5, 0), 35)

    scenario = data.Scenario(TSIM, realsensor, scene, egocar, targetlist=data.TargetCar.targetlist)

    return scenario


def getStaticScene1():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    # Initialize Testing Objects
    obj1 = StaticTarget([(10, -5), (10, 5), (16, 5), (16, -5)])  # Rectangle
    obj2 = StaticTarget([(20, -20), (20, 20), (22, 20), (22, -20)])  # Wall
    obj3 = StaticTarget([(20, -5), (20, 5), (26, 4), (26, -4)])  # Trapezoid middle moved to right
    obj4 = StaticTarget([(0, -10), (10, -3), (5, -5)])  # Lower Triangle
    obj5 = StaticTarget([(10, -5), (10, 5), (16, 4), (16, -4)])  # Trapezoid middle
    obj6 = StaticTarget([(0, 10), (10, 3), (5, 5)])  # upper Triangle

    return Scene([obj1])

def getStaticScene2():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])

    return Scene([object1, object5, object6, object7])

def getStaticScene3():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    # Initialize Testing Objects
    obj1 = StaticTarget([(10, -5), (10, 5), (16, 5), (16, -5)])  # Rectangle
    obj2 = StaticTarget([(20, -20), (20, 20), (22, 20), (22, -20)])  # Wall
    obj3 = StaticTarget([(20, -5), (20, 5), (26, 4), (26, -4)])  # Trapezoid middle moved to right
    obj4 = StaticTarget([(0, -10), (10, -3), (5, -5)])  # Lower Triangle
    obj5 = StaticTarget([(10, -5), (10, 5), (16, 4), (16, -4)])  # Trapezoid middle
    obj6 = StaticTarget([(0, 10), (10, 3), (5, 5)])  # upper Triangle
    obj7 = StaticTarget([(0, 10), (10, 3), (5, 5)])  # upper Triangle
    obj8 = StaticTarget([(-10, -5), (-10, 5), (-16, 5), (-16, -5)])  # Rectangle

    return Scene([obj8])

def getStaticScene4():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1, object5, object6, object7, object8,object9,object10])

def getStaticSceneB_0():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([])
def getStaticSceneB_1():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1])
def getStaticSceneB_2():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2])
def getStaticSceneB_3():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3])
def getStaticSceneB_4():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4])
def getStaticSceneB_5():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4,object5])
def getStaticSceneB_6():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4,object5,object6])
def getStaticSceneB_7():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4,object5,object6,object7])

def getStaticSceneB_8():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4,object5,object6,object7,object8])

def getStaticSceneB_9():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''

    #Initing some more street like objects
    object1 = StaticTarget([(5, 9),(10,5),(25,5),(25,9)])
    object2 = StaticTarget([(6,-4),(8,-6), (6,-8),(4,-6)])
    object3 = StaticTarget([(15,-4),(20,-4),(20,-9),(15,-9)])
    object4 = StaticTarget([(30,-2),(25,-2),(25,-7),(30,-7)])
    object5 = StaticTarget([(6,-9),(8,-9), (6,-13),(4,-11)])
    object6 = StaticTarget([(15,-9),(20,-9),(20,-14),(15,-14)])
    object7 = StaticTarget([(30,-9),(25,-9),(25,-14),(30,-14)])
    object8 = StaticTarget([(40, 5), (40, 20), (43, 20), (43, 5)])  # Wall
    object9 = StaticTarget([(5, 19),(10,15),(25,15),(25,19)])
    object10 = StaticTarget([(40, -10), (45, -10), (45, -15), (40, -15)])  # Rectangle


    #Create Scene with objects
    return Scene([object1,object2,object3,object4,object5,object6,object7,object8,object9])
def getStaticSceneempty():
    '''Function so set instance of Scene Class
        Output: Instance of Scene Class '''


    return Scene([])

#main function that helps setting up scenarios and scenes
def main():
    scene = getStaticScene4()
    scene.draw()

if __name__ == '__main__':
    main()