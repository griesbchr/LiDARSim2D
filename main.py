import cProfile
import io
import pstats
import sys
import time

import numpy as np

import getPointcloud
import scenarios
from coordTrans import sensor2global
from viewer import sceneViewer


def main():
    pr = cProfile.Profile()

    #####SELECT SIM PARAMETERS#####
    # Model: choose only one, commandline mode selection will override script mode selection!!
    en_real_sensor = 1  # one timestep for each lidar ray -> motion distortion included
    en_model0 = 0  # one timestep for each lidar rotation -> no motion distortion
    en_model1 = 0  # one timestep per rotation -> motion distortin via interpolation of ego vehicle
    en_model2 = 0  # one timestep per rotation -> motion distortin via interpolation of ego and target vehicle
    en_model3 = 0  # one timestep per rotation -> motion distortion via interpolation of ego vehicle and target vertices
    # Calculation Mode: choose one
    # numba           = None
    rt = 1
    #regular = 0
    # Select Scenario
    scenario_name = "Scenario_crossing"
    # scenario_name = "Scenario_only_ego"
    #scenario_name = "Scenario_Benchmark"
    #scenario_name = "Scenario_TargetLaneTurnIn"
    #scenario_name = "Scenario_debug"
    # Additional options
    #####SIMULATION PARAMETERS#####
    T_SIM = 4  # only multiples of 1/20 = 0.05 (turning time)

    profiling = 0
    viewer = 1

    #######MODE SELECTION VIA COMMANDLINE
    if len(sys.argv) > 1:
        print("Commandline Select: " + sys.argv[1])
        simmode = sys.argv[1]
        if len(sys.argv) > 3:
            scenario_name = sys.argv[2]
            T_SIM = np.float(sys.argv[3])
        #print("sysarg0", sys.argv[0])
        #print("sysarg1", sys.argv[1])
        #print("sysarg2", sys.argv[2])
        #print("sysarg3", sys.argv[3])

        en_real_sensor = 0  # setting all modes to 0
        en_model0 = 0
        en_model1 = 0
        en_model2 = 0
        en_model3 = 0

        if simmode == 'realsensor':
            en_real_sensor = 1
        elif simmode == 'model0':
            en_model0 = 1
        elif simmode == 'model1':
            en_model1 = 1
        elif simmode == "model2":
            en_model2 = 1
        elif simmode == "model3":
            en_model3 = 1
    else:
        if en_real_sensor:
            simmode = 'realsensor'
        elif en_model0:
            simmode = 'model0'
        elif en_model1:
            simmode = 'model1'
        elif en_model2:
            simmode = 'model2'
        elif en_model3:
            simmode = 'model3'

        print("Script Select: " + simmode)

    ######################Getting Scenario##########################
    func_name = "get" + scenario_name
    #try:
    getScenario = getattr(scenarios, func_name)  # Scenarios is the filename, funcname the name of function
    scenario = getScenario(T_SIM, en_real_sensor)
    #except:
    #    print("Scenario not found or error occured in getScenario function ")

    ##############################CALCULATING POINTSCLOUDS#####################
    print("Calculating..")
    PC = []
    if simmode == "realsensor":
        pointlist = []

    for time_index in np.arange(len(scenario.t)):
        if en_real_sensor:
            #    if regular:
            #        calcmode = 'regular'
            #        pointcloud = getPointcloud.getPointCloudRealSensor(*scenario.getPCargs("realsensor_fullscenatio"))
            #        break
            if rt:
                calcmode = "realtime"
                args = scenario.getPCargs("realsensor", time_index)
                PC_update = args[-1]
                pr.enable()  # for profilin
                point = getPointcloud.getPointCloudRealSensor_rt(*args[0:-1])  # leaves out last element

                pointlist.append(point)
                pr.disable()  # for profiling

                if PC_update:
                    _lidarmountx = args[6]
                    _lidarmounty = args[7]
                    ego_x_y_th = args[0]
                    pr.enable()  # for profiling
                    pointcloud = np.stack(pointlist)
                    pointcloud[:, 0:2] = sensor2global(np.stack(pointlist)[:, 0:2], _lidarmountx, _lidarmounty,
                                                       ego_x_y_th)
                    pointlist.clear()
                    pr.disable()  # for profiling

        elif en_model0:
            # if regular:
            #    calcmode = 'regular'
            #    pointcloud = getPointcloud.getPointCloudModel0(t, _n_turns, ego_x_y_th, scene.vertices, scene.line_vecs,
            #                                                egocar._n_rays, egocar.alpha_init, egocar._alpha_inc,egocar._d_max, egocar._lidarmountx,egocar._lidarmounty,
            #                                                target_x_y_th, egocar._fov,Car._lowerleft, Car._lowerright, Car._upperleft, Car._upperright)
            if rt:
                calcmode = "realtime"
                args = scenario.getPCargs("model0", time_index)
                pr.enable()  # for profiling
                pointcloud = getPointcloud.getPointCloudModel0_rt(*args)
                pr.disable()  # for profiling

        elif en_model1:
            # if regular:
            #    calcmode = 'regular'
            #    pointcloud = getPointcloud.getPointCloudModel1(t, _n_turns, ego_x_y_th, scene.vertices, scene.line_vecs,
            #                                                      egocar._n_rays, egocar.alpha_init, egocar._alpha_inc,egocar._d_max, egocar._lidarmountx,egocar._lidarmounty,
            #                                                   target_x_y_th, egocar._fov, egocar._counterclockwise, Car._lowerleft, Car._lowerright, Car._upperleft, Car._upperright)
            if rt:
                calcmode = "realtime"
                args = scenario.getPCargs("model1", time_index)
                pr.enable()  # for profiling
                pointcloud = getPointcloud.getPointCloudModel1_rt(*args)
                pr.disable()

        elif en_model2:
            # if regular:
            #    calcmode = 'regular'
            #    pointcloud = getPointcloud.getPointCloudModel2(t, _n_turns, ego_x_y_th, scene.vertices, scene.line_vecs,
            #                                                      egocar._n_rays, egocar.alpha_init, egocar._alpha_inc,egocar._d_max, egocar._lidarmountx,egocar._lidarmounty,
            #                                                      target_x_y_th, egocar._fov, egocar._counterclockwise, Car._lowerleft,Car._lowerright, Car._upperleft, Car._upperright)
            if rt:
                calcmode = "realtime"
                args = scenario.getPCargs("model2", time_index)
                pr.enable()  # for profiling
                pointcloud = getPointcloud.getPointCloudModel2_rt(*args)
                pr.disable()

        elif en_model3:
            # if regular:
            #    calcmode = 'regular'
            #    pointcloud = getPointcloud.getPointCloudModel3(t, _n_turns, ego_x_y_th, scene.vertices, scene.line_vecs,
            #                                                      egocar._n_rays, egocar.alpha_init, egocar._alpha_inc,egocar._d_max, egocar._lidarmountx,egocar._lidarmounty,
            #                                                      target_x_y_th, egocar._fov, egocar._counterclockwise, Car._lowerleft, Car._lowerright, Car._upperleft, Car._upperright)
            if rt:
                calcmode = "realtime"
                args = scenario.getPCargs("model3", time_index)
                pr.enable()  # for profiling
                pointcloud = getPointcloud.getPointCloudModel3_rt(*args)
                pr.disable()
        else:
            print("ModeSelectionError!")

        if calcmode == "realtime" and simmode != "realsensor":
            PC.append(pointcloud)
        elif calcmode == "realtime" and PC_update:
            PC.append(pointcloud)

    if rt:
        pointcloud = np.stack(PC)

    ######################CODE PROFILING#####################
    sortby = 'cumulative'
    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    if profiling:
        print(s.getvalue())

    ##################PRINTING AND LOGGING SIMULATION RESULTS###############
    exec_time = ps.total_tt
    realtimefactor = (exec_time / T_SIM)
    print('Pointcloud Calc Time = {:.6f}'.format(exec_time), 's at realtimefactor of ', realtimefactor)
    with open("Log.txt", "a") as text_file:
        print(f"{time.ctime()}; T_SIM=; {T_SIM}; PC Calctime=; {(exec_time)}; SimMode=;", simmode,
              "; Calcmode=; " + calcmode, f"; Realtimefactor=; {realtimefactor}; Scenario=;",scenario_name, file=text_file)
    print("shape PC = ", pointcloud.shape)

    ##############PLOTTING SIMULATION RESULTS################
    print("T_STEP=", scenario.T_STEP)
    if viewer:
        sceneViewer(scenario.ego_x_y_th, scenario.target_x_y_th, pointcloud, simmode, scenario.scene, scenario.egocar,
                    scenario.targetlist, scenario.T_STEP, scenesight=True)


if __name__ == '__main__':
    main()
