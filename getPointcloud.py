import numpy as np
from numpy import inf

from coordTrans import vehicle2global, vehicle2globalSingle, getTargetcarVertices, sensorpos2global, sensord2global#, getTargetcarVertices_numba
from getRayIntersections import getRayIntersection as getRayIntersection

from numba import njit, jit
from numba.types import float64, int64, int32, boolean

import cProfile


#Just needed when using the SceneViewer.py
#@njit(float64[:,:,:](float64[:], int64, float64[:,:], float64[:,:], float64[:,:], int64, int64, float64, int64, float64, int64, float64[:,:], int64, int64, float64[:], float64[:], float64[:], float64[:]))
def getPointCloudRealSensor(t,_n_turns, ego_x_y_th, scene_vertices, scene_line_vecs, _n_rays, alpha_init, _alpha_inc, _d_max, _lidarmountx, _lidarmounty, target_x_y_th, fov, counterclockwise, _lowerleft, _lowerright, _upperleft, _upperright):
    pointcloud = np.zeros((_n_turns, _n_rays, 3))  # x, y, distance (nan if no hit)
    if fov == 360:
        alpha_cw = np.arange(alpha_init+fov, alpha_init, -abs(_alpha_inc))
        alpha_ccw = np.arange(alpha_init, alpha_init + fov, abs(_alpha_inc))
    else:
        alpha_max = int(fov / 2) + alpha_init
        alpha_min = -int(fov / 2) + alpha_init
        alpha_cw = np.arange(alpha_max, alpha_min, -abs(_alpha_inc))
        alpha_ccw = np.arange(alpha_min, alpha_max, abs(_alpha_inc))

    lidar_xy = sensorpos2global(np.zeros((len(t), 2)), _lidarmountx, _lidarmounty, ego_x_y_th)

    if target_x_y_th is not None:
        lowerleft = np.repeat(_lowerleft, target_x_y_th.shape[0], axis=0)
        lowerright = np.repeat(_lowerright, target_x_y_th.shape[0], axis=0)
        upperleft = np.repeat(_upperleft, target_x_y_th.shape[0], axis=0)
        upperright = np.repeat(_upperright, target_x_y_th.shape[0], axis=0)
        target_vertices, target_line_vecs = getTargetcarVertices(lowerleft, lowerright, upperleft, upperright, target_x_y_th)

    for turn_nr in range(_n_turns):
        #Selecting and calculating alpha vector
        if counterclockwise:
            alpha = np.copy(alpha_ccw)
        else:
            alpha = np.copy(alpha_cw)
        alpha_turn = alpha + ego_x_y_th[turn_nr*_n_rays:(turn_nr+1)*_n_rays,2 ]
        alpha_cos = np.cos(np.deg2rad(alpha_turn))
        alpha_sin = np.sin(np.deg2rad(alpha_turn))
        for ray_nr in range(_n_rays):
            #Selecting vertices and line vecs
            vertices = scene_vertices
            line_vecs = scene_line_vecs
            if target_x_y_th is not None:
                #for carnumber in range(len(target_vertices)):
                vertices = np.concatenate((vertices, target_vertices[turn_nr*_n_rays+ray_nr]), axis=0)
                line_vecs = np.concatenate((line_vecs, target_line_vecs[turn_nr*_n_rays+ray_nr]), axis=0)
            #vertices = scene_vertices
            #line_vecs = scene_line_vecs
            number_of_segments = line_vecs.shape[0]
            #Calculate Pointcloud
            pointcloud[turn_nr][ray_nr] = getRayIntersection(alpha_cos[ray_nr], alpha_sin[ray_nr],
                                                                lidar_xy[turn_nr*_n_rays+ray_nr][0], lidar_xy[turn_nr*_n_rays+ray_nr][1],
                                                                _d_max, number_of_segments, vertices, line_vecs)
        #Transform hit coordinates (x_hit, y_hit and d) into global coordinates, done at last ray of turn
        pointcloud[turn_nr][:, 0:2] = sensord2global(pointcloud[turn_nr][:, 2], _lidarmountx, _lidarmounty, ego_x_y_th[turn_nr * _n_rays + ray_nr, :3], alpha)

        if fov != 360:  #reverse direction for next turn
            counterclockwise = not(counterclockwise)

    return pointcloud


def getPointCloudDocDummy(ego_x_y_th,
                            scene_vertices, s, target_vertices, target_line_vecs,
                            _n_rays, alpha_init,_fov,_alpha_inc,_lidarmountx, _lidarmounty, counterclockwise, _d_max):
    '''
    Function that only exists to doc the parameters of all getPointCloud functions.

    :param ego_x_y_th: array with ego car position data for current timestep in the global coordinate system.
                       shape (3,) where 3 stands for x,y,theta. dtype=float64
    :param scene_vertices: array of vertices of all polygone objects that make up the stationary objects in global coordinate system.
                           Can be None if no static targets exist, else it is shape (#points, 2)  where 2 stands for x and y. dtype=float64
    :param scene_line_vecs: array of line vectors of all polygone objects that make up the stationary objects in global coordinate system.
                            Can be None if no static targets exist, else it is shape (#vectors, 2)  where 2 stands for x and y. dtype=float64
    :param target_line_vecs: array of line vectors of all polygone objects that make up the moving objects in global coordinate system.
                             Can be None if no dynamic targets exist, else it isshape (#vectors, 2)  where 2 stands for x and y. dtype=float64
    :param target_vertices: array of vertices of all polygone objects that make up the moving objects in global coordinate system.
                            Can be None if no dynamic targets exist, else it is shape (#vectoes, 2)  where 2 stands for x and y. dtype=float64
    :param _n_rays: number of rays within one lidar rotation dtype=int
    :param alpha_init: angle at which lidar starts to scan, in vehicle coordinate system. dtype=float
                    if _fov < 360deg then alpha_init needs to alternate between maximal and minimal angle for each scanning frame
    :param _fov: field of view of the lidar.
    :param _alpha_inc: the angular increment of the lidar angle for each ray
    :param _lidarmountx: x coordinate of mounting position of the lidar in vehicle coordinate system. dtype=float64
    :param _lidarmounty: y coordinate of mounting position of the lidar in vehicle coordinate system. dtype=float64
    :param counterclockwise: Indicates the current lidar turning direction. If True then lidar currently turns counterclockwise. dtype=boolean
    :param _d_max: maximal distance that lidar sensor can measure in m. dtype=float64

    :return: array (x_coordinate, y_coordinate, distance) of hitpoints in hit coordinates. shape = (#n_rays, 3))
                               If nothing is hit, distance 'll be "inf" and coordinates 'll be the maximum range of lidar


    Realsensor:
    :param alpha: lidar scanning angle at position specified at ego_x_y_th in vehicle coordinate system.

    Model1:
    :param ego_x_y_th_prev: array with ego car position data for previous timestep in the global coordinate system.
                            shape (3,) where 3 stands for x,y,theta. dtype=float64

    Model2:
    :param ego_x_y_th_prev: array with ego car position data for previous timestep in the global coordinate system.
                            shape (3,) where 3 stands for x,y,theta. dtype=float64
    :param target_x_y_th: array with target car position data for current timestep in the global coordinate system.
                          shape (#targetcars,1,3) where 3 stands for x,y,theta. dtype=float64
    :param target_x_y_th_prev: array with target car position data for previous timestep in the global coordinate system.
                          shape (#targetcars,1,3) where 3 stands for x,y,theta. dtype=float64
    :param lowerleft: array of positions of the right rear vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param lowerright: array of positions of the right front vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param upperleft: array of positions of the left front vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param upperright: array of positions of the left rear vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float

    Model3_
    :param ego_x_y_th_prev: array with ego car position data for previous timestep in the global coordinate system.
                            shape (3,) where 3 stands for x,y,theta. dtype=float64
    :param target_line_vecs_prev: array of line vectors of all polygone objects that make up the moving objects in global coordinate system at the previous timestep.
                                  Can be None if no dynamic targets exist, else it is shape (#vectors, 2)  where 2 stands for x and y. dtype=float64
    :param target_vertices_prev: array of vertices of all polygone objects that make up the moving objects in global coordinate system at the previous timestep.
                                 Can be None if no dynamic targets exist, else it is shape (#vectoes, 2)  where 2 stands for x and y. dtype=float64

    '''
    pass


############################################REALSENSOR############################################################
#[(array(float64, 1d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 3d, C), array(float64, 3d, C), float64, float64, float64, float64) -> array(float64, 1d, A)]
#@njit(cache=True)
@njit(float64(float64[:],float64[:,:],float64[:,:],float64[:,:,:],float64[:,:,:], float64, float64, float64,float64))
def getPointCloudRealSensor_rt(ego_x_y_th,
                               scene_vertices, scene_line_vecs,
                               target_line_vecs, target_vertices,
                               _d_max, _lidarmountx, _lidarmounty, alpha):
    '''

    Returns the hitpoint coordinates in sensor coordinate system. In contrast to the other functions, this function
    returns only a single hitpoint per call. So for one lidar rotation consisting of eg. 900 single lidar rays, this
    is called 900 times for each rotation.
    Because usually a lidar pointcloud has the format (#lidar_turns, #rays_per_lidarturn, 3), a wrapper function needs
    concatinate the points after each lidar turn. The lidar hit points are first stored in a buffer and whenever the lidar
    has finnished its rotation the buffer is emptied and the whole rotation pointcloud can be appended to the lidar
    pointcloud array.
    The wrapper function needs to keep track of the current lidar angle alpha to do so. To emulate the real sensor, the
    wrapper function also needs to shift the hitpoint coordinates according to the position of the ego vehicle at the
    last position. This is done via the sensord2global (sensor distance to global) function.

    :param alpha: lidar scanning angle at position specified at ego_x_y_th in vehicle coordinate system.
    '''
    pointcloud = np.zeros((3))  # x, y, distance (nan if no hit)
    lidar_xy = vehicle2globalSingle(np.array([_lidarmountx, _lidarmounty]), ego_x_y_th)
    alpha_turn = alpha + ego_x_y_th[2]
    alpha_cos = np.cos(np.deg2rad(alpha_turn))
    alpha_sin = np.sin(np.deg2rad(alpha_turn))
    if scene_vertices is not None:
        if target_vertices is not None: #dynamic and static
            vertices = np.concatenate((scene_vertices, target_vertices[0]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_line_vecs[0]), axis=0)
        else: #only static
            vertices = scene_vertices
            line_vecs = scene_line_vecs
    else:
        if target_vertices is not None: #only dynamic
            vertices = target_vertices[0]
            line_vecs = target_line_vecs[0]
        else: #no target
            return inf

    number_of_segments = line_vecs.shape[0]
    dist = getRayIntersection(alpha_cos, alpha_sin, lidar_xy[0], lidar_xy[1], _d_max, number_of_segments, vertices, line_vecs)
    return dist

############################################MODEL0############################################################
#[(array(float64, 1d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 3d, C), array(float64, 3d, C), int64, float64, int64, float64, float64, float64, float64) -> array(float64, 2d, C)]
#@njit(cache=True)
@njit(float64[:,:](float64[:],float64[:,:],float64[:,:],float64[:,:,:],float64[:,:,:],  int64, float64, int64, float64, float64, float64, float64))
def getPointCloudModel0_rt(ego_x_y_th,
                           scene_vertices, scene_line_vecs,
                           target_vertices,target_line_vecs,
                           _n_rays, alpha_init, _fov,_alpha_inc, _lidarmountx, _lidarmounty,_d_max):
    '''
    That fastest lidar model, which doesnt take motion distortion into account. The ray intersections for a whole lidar
    turn are calculated using the positions specified in the arguments. That means, that the ray intersections at the
    start of the turn are rather different compared to the real sensor because the ego and target vehicles movement
    before the calculation of the intersections are not taken into account.
    No interpolation is used to modulate the motion distortion.

    '''
    pointcloud = np.zeros((_n_rays, 3))  # x, y, distance
    alpha_sensor = np.arange(alpha_init, alpha_init+_fov, abs(_alpha_inc))
    alpha = alpha_sensor + ego_x_y_th[2]
    alpha_cos = np.cos(np.deg2rad(alpha))
    alpha_sin = np.sin(np.deg2rad(alpha))
    lidar_xy = vehicle2globalSingle(np.array([_lidarmountx, _lidarmounty]), ego_x_y_th)

    if scene_vertices is not None:
        if target_vertices is not None: #dynamic and static
            vertices = np.concatenate((scene_vertices, target_vertices[0]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_line_vecs[0]), axis=0)
        else: #only static
            vertices = scene_vertices
            line_vecs = scene_line_vecs
    else:
        if target_vertices is not None: #only dynamic
            vertices = target_vertices[0]
            line_vecs = target_line_vecs[0]
        else: #no target
            pointcloud[:,0] = _d_max*alpha_cos
            pointcloud[:,1] = _d_max*alpha_sin
            pointcloud[:,2] = inf
            pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
            return pointcloud

    number_of_segments = line_vecs.shape[0]
    for ray_nr in range(_n_rays):
        pointcloud[ray_nr,2] = getRayIntersection(alpha_cos[ray_nr],alpha_sin[ray_nr], lidar_xy[0], lidar_xy[1], _d_max, number_of_segments, vertices, line_vecs)

    pointcloud[:,0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)

    return pointcloud

############################################MODEL1############################################################
#[(array(float64, 1d, C), array(float64, 3d, C), array(float64, 1d, C), array(float64, 3d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), int64, float64, int64, float64, float64, float64, bool, float64) -> array(float64, 2d, C)]
#@jit(float64[:,:](float64[:],float64[:,:,:], float64[:],float64[:,:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:], int64, float64, int64, float64, float64, float64, boolean, float64))
#@njit(cache=True)
@njit(float64[:,:](float64[:], float64[:],float64[:,:],float64[:,:],float64[:,:,:],float64[:,:,:], int32,float64, int32,float64, float64,float64,boolean, float64))
def getPointCloudModel1_rt(ego_x_y_th, ego_x_y_th_prev,
                            scene_vertices, scene_line_vecs, target_vertices, target_line_vecs,
                            _n_rays, alpha_init,_fov,_alpha_inc,_lidarmountx, _lidarmounty, counterclockwise, _d_max):
    '''
    Ego position interpolation:
    Calculates the pointcould of one lidar rotation using an interpolation of the ego vehicles position between
    the start and the end of the rotation (which is also the start of the next rotation).
    Because ego movement is taken into account, the model is very accurate for static targets and slow moving dynamic
    targets. The movements of dynamic targets are disregarded using this model.
    '''

    pointcloud = np.zeros((_n_rays, 3))  # x, y, distance (nan if no hit)
    ego_x_y_th_inter = np.zeros((_n_rays, 3))
    #Interpolation ego position
    ego_x_y_th_inter[:,0] = np.linspace(ego_x_y_th_prev[0], ego_x_y_th[0], _n_rays)
    ego_x_y_th_inter[:,1] = np.linspace(ego_x_y_th_prev[1], ego_x_y_th[1], _n_rays)
    ego_x_y_th_inter[:,2] = np.linspace(ego_x_y_th_prev[2], ego_x_y_th[2], _n_rays)
    lidar_xy = vehicle2global(np.array([_lidarmountx, _lidarmounty]), ego_x_y_th_inter) #model 1 -> lidar_xy is f(ray_nr)
    if counterclockwise:
        alpha_sensor = np.arange(alpha_init, alpha_init + _fov, abs(_alpha_inc))
    else:
        alpha_sensor = np.arange(alpha_init, alpha_init-_fov, -abs(_alpha_inc))
    alpha = alpha_sensor + ego_x_y_th_inter[:,2 ]
    alpha_rad = np.deg2rad(alpha)
    alpha_cos = np.cos(alpha_rad)
    alpha_sin = np.sin(alpha_rad)

    if scene_vertices is not None:
        if target_vertices is not None: #dynamic and static
            vertices = np.concatenate((scene_vertices, target_vertices[0]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_line_vecs[0]), axis=0)
        else: #only static
            vertices = scene_vertices
            line_vecs = scene_line_vecs
    else:
        if target_vertices is not None: #only dynamic
            vertices = target_vertices[0]
            line_vecs = target_line_vecs[0]
        else: #no target
            pointcloud[:, 0] = _d_max * alpha_cos
            pointcloud[:, 1] = _d_max * alpha_sin
            pointcloud[:, 2] = inf
            pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
            return pointcloud

    number_of_segments = line_vecs.shape[0]
    for ray_nr in range(_n_rays):
        pointcloud[ray_nr,2] = getRayIntersection(alpha_cos[ray_nr],alpha_sin[ray_nr], lidar_xy[ray_nr][0], lidar_xy[ray_nr][1], _d_max, number_of_segments, vertices, line_vecs)  #model2 lidar_xy and vertices+linevecs are f(ray_nr)
    pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)

    return pointcloud

############################################MODEL2############################################################
#[(array(float64, 1d, C), array(float64, 3d, C), array(float64, 1d, C), array(float64, 3d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 2d, C), int64, float64, int64, float64, float64, float64, bool, float64) -> array(float64, 2d, C)]
#@njit(cache=True)
@njit(float64[:,:](float64[:],float64[:,:,:], float64[:],float64[:,:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:], int64, float64, int64, float64, float64, float64, boolean, float64))
def getPointCloudModel2_rt(ego_x_y_th,target_x_y_th, ego_x_y_th_prev, target_x_y_th_prev,
                            lowerleft, lowerright, upperleft, upperright,
                            scene_vertices, scene_line_vecs,
                            _n_rays, alpha_init,_fov,_alpha_inc,_lidarmountx, _lidarmounty, counterclockwise, _d_max):
    '''
    Egoposition and targetposition interpolation:
    Calculates the pointcould of one lidar rotation using an interpolation of the ego vehicle position (same as model1)
    and also interpolation of the target vehicles positions.


    :param ego_x_y_th_prev: array with ego car position data for previous timestep in the global coordinate system.
                            shape (3,) where 3 stands for x,y,theta. dtype=float64
    :param target_x_y_th: array with target car position data for current timestep in the global coordinate system.
                          shape (#targetcars,1,3) where 3 stands for x,y,theta. dtype=float64
    :param target_x_y_th_prev: array with target car position data for previous timestep in the global coordinate system.
                          shape (#targetcars,1,3) where 3 stands for x,y,theta. dtype=float64
    :param lowerleft: array of positions of the right rear vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param lowerright: array of positions of the right front vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param upperleft: array of positions of the left front vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    :param upperright: array of positions of the left rear vertices of the target cars, in car coordinate system.
                      shape(#targetcars, 2), dtype=float
    '''
    pointcloud = np.zeros((_n_rays, 3))  # x, y, distance (nan if no hit)
    ego_x_y_th_inter = np.zeros((_n_rays, 3))
    if target_x_y_th is not None:
        carnumber,_,_ = target_x_y_th.shape
        target_x_y_th_inter = np.zeros((carnumber, _n_rays,3))
    #Interpolation of ego position
    ego_x_y_th_inter[:,0] = np.linspace(ego_x_y_th_prev[0], ego_x_y_th[0], _n_rays)
    ego_x_y_th_inter[:,1] = np.linspace(ego_x_y_th_prev[1], ego_x_y_th[1], _n_rays)
    ego_x_y_th_inter[:,2] = np.linspace(ego_x_y_th_prev[2], ego_x_y_th[2], _n_rays)
    lidar_xy = vehicle2global(np.array([_lidarmountx, _lidarmounty]), ego_x_y_th_inter)
    ##vertices = np.repeat(np.expand_dims(scene_vertices, axis=0), _n_rays, axis=0)
    ##line_vecs = np.repeat(np.expand_dims(scene_line_vecs, axis=0), _n_rays, axis=0)
    if counterclockwise:
        alpha_sensor = np.arange(alpha_init, alpha_init+_fov, abs(_alpha_inc))
    else:
        alpha_sensor = np.arange(alpha_init, alpha_init-_fov, -abs(_alpha_inc))
    alpha = alpha_sensor + ego_x_y_th_inter[:,2]
    alpha_rad = np.deg2rad(alpha)
    alpha_cos = np.cos(alpha_rad)
    alpha_sin = np.sin(alpha_rad)

    if scene_vertices is not None:
        if target_x_y_th is not None: #dynamic and static
            for i in range(carnumber):
                target_x_y_th_inter[i, :, 0] = np.linspace(target_x_y_th_prev[i][0][0], target_x_y_th[i][0][0], _n_rays)
                target_x_y_th_inter[i, :, 1] = np.linspace(target_x_y_th_prev[i][0][1], target_x_y_th[i][0][1], _n_rays)
                target_x_y_th_inter[i, :, 2] = np.linspace(target_x_y_th_prev[i][0][2], target_x_y_th[i][0][2], _n_rays)
            target_vertices, target_line_vecs = getTargetcarVertices(lowerleft, lowerright, upperleft, upperright,
                                                                     target_x_y_th_inter[:])

            vertices = np.concatenate((scene_vertices, target_vertices[0]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_line_vecs[0]), axis=0)
        else: #only static
            vertices = scene_vertices
            line_vecs = scene_line_vecs
    else:
        if target_x_y_th is not None: #only dynamic
            for i in range(carnumber):
                target_x_y_th_inter[i, :, 0] = np.linspace(target_x_y_th_prev[i][0][0], target_x_y_th[i][0][0], _n_rays)
                target_x_y_th_inter[i, :, 1] = np.linspace(target_x_y_th_prev[i][0][1], target_x_y_th[i][0][1], _n_rays)
                target_x_y_th_inter[i, :, 2] = np.linspace(target_x_y_th_prev[i][0][2], target_x_y_th[i][0][2], _n_rays)
            target_vertices, target_line_vecs = getTargetcarVertices(lowerleft, lowerright, upperleft, upperright,
                                                                     target_x_y_th_inter[:])

            vertices = target_vertices[0]
            line_vecs = target_line_vecs[0]
        else: #no target
            pointcloud[:, 0] = _d_max * alpha_cos
            pointcloud[:, 1] = _d_max * alpha_sin
            pointcloud[:, 2] = inf
            pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
            return pointcloud

        #vertices = np.concatenate((vertices, target_vertices), axis=1)
        #line_vecs = np.concatenate((line_vecs, target_line_vecs),axis=1)
        ##vertices = np.concatenate((np.broadcast_to(vertices,(egocar._n_rays, carnumber*4,2)), target_vertices), axis=1)
        ##line_vecs = np.concatenate((np.broadcast_to(line_vecs,(egocar._n_rays, carnumber*4,2)), target_line_vecs), axis=1)
    #number_of_segments = line_vecs.shape[1]

    for ray_nr in range(_n_rays):
        if target_x_y_th is not None and scene_vertices is not None:
            vertices = np.concatenate((scene_vertices, target_vertices[ray_nr]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_line_vecs[ray_nr]), axis=0)
        elif target_x_y_th is not None:
            vertices = target_vertices[ray_nr]
            line_vecs = target_line_vecs[ray_nr]
        number_of_segments = line_vecs.shape[0]
        pointcloud[ray_nr,2] = getRayIntersection(alpha_cos[ray_nr], alpha_sin[ray_nr], lidar_xy[ray_nr][0],
                                                lidar_xy[ray_nr][1], _d_max, number_of_segments, vertices, line_vecs)
    pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
    return pointcloud

############################################MODEL3############################################################
#[(array(float64, 1d, C), array(float64, 1d, C), array(float64, 2d, C), array(float64, 2d, C), array(float64, 3d, C), array(float64, 3d, C), array(float64, 3d, C), array(float64, 3d, C), int64, float64, int64, float64, float64, float64, bool, float64) -> array(float64, 2d, C)]
#@njit(cache=True)
@njit(float64[:,:](float64[:], float64[:],float64[:,:],float64[:,:],float64[:,:,:],float64[:,:,:],float64[:,:,:],float64[:,:,:], int32,float64, int32,float64, float64,float64,boolean, float64))
def getPointCloudModel3_rt(ego_x_y_th, ego_x_y_th_prev,
                           scene_vertices, scene_line_vecs,
                           target_vertices,target_line_vecs, target_vertices_prev, target_line_vecs_prev,
                           _n_rays, alpha_init, _fov,_alpha_inc,_lidarmountx, _lidarmounty, counterclockwise, _d_max):
    '''
    Egoposition and targets vertices interpolation:
    Calculates the pointcloud for one lidar rotation using interpolation for the ego position for each lidar ray
    and interpolation of the targets vertices.

    :param target_line_vecs_prev: array of line vectors of all polygone objects that make up the moving objects in global coordinate system at the previous timestep.
                                  Can be None if no dynamic targets exist, else it isshape (#vectors, 2)  where 2 stands for x and y. dtype=float64
    :param target_vertices_prev: array of vertices of all polygone objects that make up the moving objects in global coordinate system at the previous timestep.
                                 Can be None if no dynamic targets exist, else it is shape (#vectoes, 2)  where 2 stands for x and y. dtype=float64
    '''
    #Initialization of varables
    pointcloud = np.zeros((_n_rays, 3))
    ego_x_y_th_inter = np.zeros((_n_rays, 3))
    if target_vertices is not None:
        _,carlinenumber,_ = target_vertices.shape
        target_vertices_inter = np.zeros((_n_rays, carlinenumber, 2))
        target_linevecs_inter = np.zeros((_n_rays, carlinenumber, 2))

    #Interpolation ego position
    ego_x_y_th_inter[:,0] = np.linspace(ego_x_y_th_prev[0], ego_x_y_th[0], _n_rays)
    ego_x_y_th_inter[:,1] = np.linspace(ego_x_y_th_prev[1], ego_x_y_th[1], _n_rays)
    ego_x_y_th_inter[:,2] = np.linspace(ego_x_y_th_prev[2], ego_x_y_th[2], _n_rays)

    #Calculaion of angle alpha
    if counterclockwise:
        alpha_sensor = np.arange(alpha_init, alpha_init+_fov, abs(_alpha_inc))
    else:
        alpha_sensor = np.arange(alpha_init, alpha_init-_fov, -abs(_alpha_inc))
    alpha = alpha_sensor + ego_x_y_th_inter[:,2 ]
    alpha_rad = np.deg2rad(alpha)
    alpha_cos = np.cos(alpha_rad)
    alpha_sin = np.sin(alpha_rad)

    lidar_xy = vehicle2global(np.array([_lidarmountx, _lidarmounty]), ego_x_y_th_inter)

    #Interpolation of dynamic targets vertices
    if scene_vertices is not None:
        if target_vertices is not None: #dynamic and static
            for i in range(carlinenumber):
                target_vertices_inter[:, i, 0] = np.linspace(target_vertices_prev[0,i,0], target_vertices[0,i,0], _n_rays)
                target_vertices_inter[:, i, 1] = np.linspace(target_vertices_prev[0,i,1], target_vertices[0,i,1], _n_rays)
                target_linevecs_inter[:, i, 0] = np.linspace(target_line_vecs_prev[0,i,0], target_line_vecs[0,i,0], _n_rays)
                target_linevecs_inter[:, i, 1] = np.linspace(target_line_vecs_prev[0,i,1], target_line_vecs[0,i,1], _n_rays)
        else: #only static
            vertices = scene_vertices
            line_vecs = scene_line_vecs
            number_of_segments = line_vecs.shape[0]
    else:
        if target_vertices is not None: #only dynamic
            for i in range(carlinenumber):
                target_vertices_inter[:, i, 0] = np.linspace(target_vertices_prev[0,i,0], target_vertices[0,i,0], _n_rays)
                target_vertices_inter[:, i, 1] = np.linspace(target_vertices_prev[0,i,1], target_vertices[0,i,1], _n_rays)
                target_linevecs_inter[:, i, 0] = np.linspace(target_line_vecs_prev[0,i,0], target_line_vecs[0,i,0], _n_rays)
                target_linevecs_inter[:, i, 1] = np.linspace(target_line_vecs_prev[0,i,1], target_line_vecs[0,i,1], _n_rays)

        else: #no target
            pointcloud[:, 0] = _d_max * alpha_cos
            pointcloud[:, 1] = _d_max * alpha_sin
            pointcloud[:, 2] = inf
            pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
            return pointcloud

    for ray_nr in range(_n_rays):
        if target_vertices is not None and scene_vertices is not None:
            vertices = np.concatenate((scene_vertices, target_vertices_inter[ray_nr]), axis=0)
            line_vecs = np.concatenate((scene_line_vecs, target_linevecs_inter[ray_nr]), axis=0)
        else:
            vertices = target_vertices_inter[ray_nr]
            line_vecs = target_linevecs_inter[ray_nr]

        number_of_segments = line_vecs.shape[0]
        pointcloud[ray_nr,2] = getRayIntersection(alpha_cos[ray_nr],alpha_sin[ray_nr], lidar_xy[ray_nr][0], lidar_xy[ray_nr][1], _d_max, number_of_segments, vertices, line_vecs)
    pointcloud[:, 0:2] = sensord2global(pointcloud[:, 2], _lidarmountx, _lidarmounty, ego_x_y_th, alpha_sensor)
    return pointcloud


