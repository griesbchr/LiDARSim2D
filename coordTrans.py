import numpy as np
import matplotlib.pyplot as plt

from numba import njit
from numba.types import  float64, int64, int32, UniTuple



@njit(float64[:,:](float64[:],float64[:,:]))
def vehicle2global(vehicle_coordinates, egocar_x_y_th):
    '''
        Transforms one set of coordinates from vehicle coordinate system to a global coordinate system for all the
        positions given by the egocar_x_y_th argument. If transformation needs to be done for just one egocar_x_y_th
        position, use the vehicle2globalSingle function.

        Used to transform the position of the lidar, given in vehicle coordinates, to global coordinates
        for the calculation of the rayintersection.
        Since the position of the lidar stays the same during a simulation, the vehicle coordinates array just needs to
        be one dimentional.
    :param vehicle_coordinates: Array of vehicle coordinates. shape=(2,) (where 2 are x and y coordinates), dtype=float
    :param egocar_x_y_th: Array of egocar position. shape=(#points, 3) (where 3 are x,y,theta), dtype=float
    :return:
        global_coordinates: Array of global coordinates. shape=(#points,2) (where 2 are x and y), dtype=float
    '''

    global_coordinates = np.empty_like((egocar_x_y_th[:,:2]), dtype= np.float64)

    m_trans = np.empty((2,2,global_coordinates.shape[0]))
    for i in range(global_coordinates.shape[0]):    #for each sensor point, eg. each lidar rotation
        angle_rad = np.deg2rad(egocar_x_y_th[i,2])
        m_trans[:,:,i] = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],    #has shape (2,2,#points) one transformation matrix for each point
                                   [np.sin(angle_rad), np.cos(angle_rad)]])
        global_coordinates[i] =  egocar_x_y_th[i,0:2] +  (np.ascontiguousarray(m_trans[:,:,i]) @ vehicle_coordinates)

    return global_coordinates

@njit(float64[:](float64[:],float64[:]))
def vehicle2globalSingle(vehicle_coordinates, egocar_x_y_th):
    '''
        Transforms one set of coordinates from vehicle coordinate system to a global coordinate system for one position
        given by the egocar_x_y_th argument. If transformation needs to be done for just multiple egocar_x_y_th
        positions, use the vehicle2global function.

        Used to transform the position of the lidar, given in vehicle coordinates, to global coordinates
        for the calculation of the rayintersection.
    :param vehicle_coordinates: Array of vehicle coordinates. shape=(2,) (where 2 are x and y coordinates), dtype=float
    :param egocar_x_y_th: Array of egocar position. shape=(3,) (where 3 are x,y,theta), dtype=float
    :return:
        global_coordinates: Array of global coordinates. shape=(2,) (where 2 are x and y), dtype=float
    '''

    global_coordinates = np.empty_like((vehicle_coordinates), dtype= np.float64)

    m_trans = np.empty((2,2))

    angle_rad = np.deg2rad(egocar_x_y_th[2])
    m_trans[:,:] = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],    #has shape (2,2,#points) one transformation matrix for each point
                               [np.sin(angle_rad), np.cos(angle_rad)]])
    global_coordinates =  egocar_x_y_th[0:2] +  np.ascontiguousarray(m_trans[:,:]) @ vehicle_coordinates

    return global_coordinates


@njit(float64[:,:](float64[:,:],float64,int64,float64[:]))
def sensor2global(hit_coordinates, _lidarmountx, _lidarmounty, egocar_x_y_th):
    '''
    Transforms array of coordinates from sensor coordinate system to global coordinate system given
    one position and one lidar mounting position.

    :param hit_coordinates: Array of coordinates in sensor coordinate system. shape=(#coordinates, 2), dtype=float
    :param _lidarmountx: x coordinate of mounting position of lidar. scalar, dtype=float
    :param _lidarmounty: y coordinate of mounting position of lidar. scalar, dtype=float
    :param egocar_x_y_th: Array of egocar position. shape=(3,) (where 3 are x,y,theta), dtype=float
    :return:
        global_coordinates: Array of global coordinates. shape=(#coordinates,2) (where 2 are x and y), dtype=float
    '''
    global_coordinates = np.empty_like((hit_coordinates), dtype=np.float64)
    angle_rad = np.deg2rad(egocar_x_y_th[2])
    m_trans = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                        [np.sin(angle_rad), np.cos(angle_rad)]])

    lidarmount_vehicle = np.empty((2,1))
    lidarmount_vehicle[0] = _lidarmountx
    lidarmount_vehicle[1] = _lidarmounty
    lidarmount_global = m_trans @ lidarmount_vehicle

    for i in range(hit_coordinates.shape[0]):    #for each sensor point, eg. each lidar rotation
        global_coordinates[i] =  lidarmount_global[:,0] + hit_coordinates[i] + egocar_x_y_th[:2]

    return global_coordinates

@njit(UniTuple(float64[:,:,:],2)(float64[:,:],float64[:,:],float64[:,:],float64[:,:],float64[:,:,:]))
def getTargetcarVertices(lowerleft, lowerright, upperleft, upperright, target_x_y_th):
    '''
    Transforms targets vertices from vehicle coordinate system to global coordinate system given the position of the
    target (x,y,theta in rad) in the global coordinate system for an arbitrary number of timesteps.
    :param lowerleft: Coordinates of lower left point in targets coordinate system. shape=(2,#targets), dtype=float
    :param lowerright: Coordinates of lower left point in targets coordinate system. shape=(2,#targets), dtype=float
    :param upperleft: Coordinates of lower left point in targets coordinate system. shape=(2,#targets), dtype=float
    :param upperright: Coordinates of lower left point in targets coordinate system. shape=(2,#targets), dtype=float
    :param target_x_y_th: Coordinates and angle of target. shape=(#targets, #timesteps, 3), dtype=float
    NOTE: if only one timestep then shape of target_x_y_th has to be (#targets,1, 3) and not just
          (#targets, 3)
    :return: Tuple(vertices, line_vecs)
        vertices: coordinates of targets vertices in global coordinate system. shape(#timesteps, 4*#Targets, 2), dtype=float
        line_vecs: line vectors for all lines of the target. shape(#timesteps, 4*#Targets, 2), dtype=float
    NOTE: line vectors are calculated by simply subtracting the vertices from each other.
    '''

    targetnumber,timesteps,_ = target_x_y_th.shape

    vertices = np.zeros((timesteps,targetnumber*4,2))  #,dtype=float
    line_vecs = np.zeros((timesteps,targetnumber*4,2)) #,dtype=float

    for targetindex in range(targetnumber):
        angle_rad = np.deg2rad(target_x_y_th[targetindex,:,2])
        for i in range(timesteps):
            m_trans = np.array([[np.cos(angle_rad[i]), -np.sin(angle_rad[i])],
                                [np.sin(angle_rad[i]), np.cos(angle_rad[i])]])
            shift_ll = m_trans @ lowerleft[targetindex]
            shift_lr = m_trans @ lowerright[targetindex]
            shift_ul = m_trans @ upperleft[targetindex]
            shift_ur = m_trans @ upperright[targetindex]
            vertices[i][targetindex*4+0]= target_x_y_th[targetindex][i][0:2]+shift_ll
            vertices[i][targetindex*4+1]= target_x_y_th[targetindex][i][0:2]+shift_lr
            vertices[i][targetindex*4+2]= target_x_y_th[targetindex][i][0:2]+shift_ur
            vertices[i][targetindex*4+3]= target_x_y_th[targetindex][i][0:2]+shift_ul
            line_vecs[i][targetindex*4+0] = vertices[i][targetindex*4+1]-vertices[i][targetindex*4+0]
            line_vecs[i][targetindex*4+1] = vertices[i][targetindex*4+2]-vertices[i][targetindex*4+1]
            line_vecs[i][targetindex*4+2] = vertices[i][targetindex*4+3]-vertices[i][targetindex*4+2]
            line_vecs[i][targetindex*4+3] = vertices[i][targetindex*4+0]-vertices[i][targetindex*4+3]

    return vertices, line_vecs

#needed to run viewer and SceneViewer
def vehicle2globalNoJit(sensor_coordinates, _lidarmountx, _lidarmounty, egocar_x_y_th):
    '''
    inputs:
    sensor_coordinate: np.array of shape (#points,2) if #points = 1 then array has to be shape(1,3)!!!!! NOT (3,)
    _lidarmountx: float
    _lidatmounty: float
    egocar_pos: np.array of shape (#points, 3), #points must be same as in sensor coords!

    NOTE: #if number of points of sensor_coordinates and ego_x_y_th are the same, then each sensor_coordinate has
            its own x_y_th. If #points of egocar_x_y_th is 1, then it is assumed that all of the sensor coordinates are
            measured at that one point.
    '''
    if egocar_x_y_th.shape[0] == 1 and egocar_x_y_th.shape[0] < sensor_coordinates.shape[0]:
        egocar_x = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,0])
        egocar_y = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,1])
        egocar_th = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,2])
        egocar_x_y_th = np.concatenate((egocar_x ,egocar_y, egocar_th), axis=1)
    vehicle_coordinates = np.empty_like((sensor_coordinates), dtype= np.float64)
    global_coordinates = np.empty_like((sensor_coordinates), dtype= np.float64)
    vehicle_coordinates[:,0] = sensor_coordinates[:,0] + _lidarmountx
    vehicle_coordinates[:,1] = sensor_coordinates[:,1] + _lidarmounty

    m_trans = np.empty((2,2,egocar_x_y_th.shape[0]))
    for i in range(egocar_x_y_th.shape[0]):
        angle_rad = np.deg2rad(egocar_x_y_th[i,2])
        m_trans[:,:,i] = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],    #has shape (2,2,#points) one transformation matrix for each point
                                   [np.sin(angle_rad), np.cos(angle_rad)]])
    for i in range(sensor_coordinates.shape[0]):    #for each sensor point, eg. each lidar rotation
        global_coordinates[i] =  egocar_x_y_th[i,0:2] +  m_trans[:,:,i] @ vehicle_coordinates[i]

    return global_coordinates

#needed to run older versions
#@njit(float64[:,:](float64[:,:],float64,int64,float64[:,:]))
def sensorpos2global(sensor_coordinates, _lidarmountx, _lidarmounty, egocar_x_y_th):
    '''
    inputs:
    sensor_coordinate: np.array of shape (#points,2) if #points = 1 then array has to be shape(1,3)!!!!! NOT (3,)
    _lidarmountx: float
    _lidatmounty: float
    egocar_pos: np.array  shape (#points, 3), #points must be same as in sensor coords!

    NOTE: #if number of points of sensor_coordinates and ego_x_y_th are the same, then each sensor_coordinate has
            its own x_y_th. If #points of egocar_x_y_th is 1, then it is assumed that all of the sensor coordinates are
            measured at that one point.
    '''
    #test1 = np.full((sensor_coordinates.shape[0],1),egocar_x_y_th[:,0])
    if egocar_x_y_th.shape[0] == 1 and egocar_x_y_th.shape[0] < sensor_coordinates.shape[0]:
        egocar_x = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,0])
        egocar_y = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,1])
        egocar_th = np.full((sensor_coordinates.shape[0], 1), egocar_x_y_th[0,2])
        egocar_x_y_th = np.concatenate((egocar_x ,egocar_y, egocar_th), axis=1)
    vehicle_coordinates = np.empty_like((sensor_coordinates), dtype= np.float64)
    global_coordinates = np.empty_like((sensor_coordinates), dtype= np.float64)
    vehicle_coordinates[:,0] = sensor_coordinates[:,0] + _lidarmountx
    vehicle_coordinates[:,1] = sensor_coordinates[:,1] + _lidarmounty

    m_trans = np.empty((2,2,egocar_x_y_th.shape[0]))
    for i in range(egocar_x_y_th.shape[0]):
        angle_rad = np.deg2rad(egocar_x_y_th[i,2])
        m_trans[:,:,i] = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],    #has shape (2,2,#points) one transformation matrix for each point
                                   [np.sin(angle_rad), np.cos(angle_rad)]])
    for i in range(sensor_coordinates.shape[0]):    #for each sensor point, eg. each lidar rotation
        global_coordinates[i] =  egocar_x_y_th[i,0:2] +  np.ascontiguousarray(m_trans[:,:,i]) @ vehicle_coordinates[i]

    return global_coordinates

#needed to run or older versions
#njit(float64[:,:](float64[:,:],float64,float64,float64[:,:]))
def vehicle2global_old(vehicle_coordinates, _lidarmountx, _lidarmounty, egocar_x_y_th):
    '''
    Transforms an array of coordinates from vehicle coordinate system to a global coordinate system.
    inputs:
    sensor_coordinate: np.array of shape (#points,2) if #points = 1 then array has to be shape(1,2)!!!!! NOT (2,)
    _lidarmountx: float (vehicle coordinate system)
    _lidatmounty: float (vehicle coordinate system)
    egocar_pos: np.array of shape (#points, 3) (global coordinate system)

    NOTE: if number of points of sensor_coordinates and ego_x_y_th are the same, then each sensor_coordinate has
            its own x_y_th. If #points of egocar_x_y_th is 1, then it is assumed that all of the sensor coordinates are
            measured at that one point.
    '''
    if egocar_x_y_th.shape[0] == 1 and egocar_x_y_th.shape[0] < vehicle_coordinates.shape[0]:
        egocar_x = np.full((vehicle_coordinates.shape[0], 1), egocar_x_y_th[0, 0])
        egocar_y = np.full((vehicle_coordinates.shape[0], 1), egocar_x_y_th[0, 1])
        egocar_th = np.full((vehicle_coordinates.shape[0], 1), egocar_x_y_th[0, 2])
        egocar_x_y_th = np.concatenate((egocar_x ,egocar_y, egocar_th), axis=1)
    vehicle_coordinates_123 = np.empty_like((vehicle_coordinates), dtype= np.float64)
    global_coordinates = np.empty_like((vehicle_coordinates), dtype= np.float64)
    vehicle_coordinates_123[:,0] = vehicle_coordinates[:, 0] + _lidarmountx
    vehicle_coordinates_123[:,1] = vehicle_coordinates[:, 1] + _lidarmounty

    m_trans = np.empty((2,2,egocar_x_y_th.shape[0]))
    for i in range(egocar_x_y_th.shape[0]):
        angle_rad = np.deg2rad(egocar_x_y_th[i,2])
        m_trans[:,:,i] = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],    #has shape (2,2,#points) one transformation matrix for each point
                                   [np.sin(angle_rad), np.cos(angle_rad)]])
    for i in range(vehicle_coordinates.shape[0]):    #for each sensor point, eg. each lidar rotation
        global_coordinates[i] =  egocar_x_y_th[i,0:2] +  np.ascontiguousarray(m_trans[:,:,i]) @ vehicle_coordinates_123[i]

    return global_coordinates
