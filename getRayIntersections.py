import numpy as np
from numpy import inf

from numba import njit, float64,float32, int32, int64   #short for @jit(nopython = True)
from numba import cfunc
@njit(float64(float64, float64, float64, float64, float64, int64, float64[:,:], float64[:,:]))
def getRayIntersection(COSALPHA, SINALPHA, lidar_x, lidar_y, _d_max, num_segs, vertices, line_vecs):
    '''
    Calculates the closest ray intersection of a lidar ray given by the lidar position and ray angle and an array of line
    segments specified by its vertices and line vectors.

    :param COSALPHA: cosinus of the current lidar angle in global coordinate system (thus including the vehicle angle), float
    :param SINALPHA: sinus of the current lidar angle in global coordinate system (thus including the vehicle angle) , float
    :param lidar_x: x coordinate of position of lidar in global coordinate system. float
    :param lidar_y: y coordinate of position of lidar in global coordinate system. float
    :param _d_max: maximum range of the lidar. float
    :param num_segs: number of lines (line segments) the ray intersection calculation is done for. int
    :param vertices: array of global coordinates specifying the vertices of the line segments. shape=(num_segs, 2), dtype=float
    :param line_vecs: array of line vectors specifying the line segments.  shape=(num_segs, 2), dtype=float
    :return:
        d: distance d between sensor and hitpoint. dtype=float
        If no target is hit, the distance will be "np.inf".
    '''

    EPSILON = 1.19209e-02
    a = np.array([-SINALPHA,COSALPHA])
    b = np.array([SINALPHA,COSALPHA])

    #Calc c Vector
    c = np.zeros((num_segs, 2))
    for i in range(num_segs):
        c[i,0] = -lidar_x + vertices[i][0]
        c[i,1] = lidar_y - vertices[i][1]

    #Calculate intersections of all non parallel lines
    x = np.zeros((num_segs, 2))
    cur_idx = 0
    for i in range(num_segs):
        det = np.dot(a, np.ascontiguousarray(line_vecs[i]))     #det is the determinant of the A matrix
        #det = np.dot(a, line_vecs[i])     #det is the determinant of the A matrix
        if (np.abs(det) > EPSILON):
            x[cur_idx,0] = np.dot(np.ascontiguousarray(line_vecs[i][::-1]), c[i])/det    #d
            #x[cur_idx,0] = np.dot(line_vecs[i][::-1], c[i])/det    #d
            x[cur_idx,1] = np.dot(b, c[i])/det     #lambda, [::-1] flips array
            cur_idx += 1

    #Sort out intersections that dont meet constraints
    valid_idx = (x[0:cur_idx,1]>0) & (x[0:cur_idx,1]<1) & (x[0:cur_idx,0]>0) & (x[0:cur_idx,0]<_d_max)
    x_valid = np.copy(x[0:cur_idx,0])
    x_valid[~valid_idx] = inf

    if np.all(~valid_idx):
        return inf #No intersection at all
        #hit_idx = -1
    else:
        hit_idx = np.argmin(x_valid)
        return x[hit_idx,0]

    #if hit_idx != -1:
    #    dist = x[hit_idx,0]
    #else:
    #    dist = inf

    #return dist

    #Calc Coordinates in Hit Coordinate System
    #p_hit = np.empty((3,))
    #if hit_idx != -1:
    #    p_hit[0] = COSALPHA*x[hit_idx,0]
    #    p_hit[1] = SINALPHA*x[hit_idx,0]
    #    p_hit[2] = x[hit_idx,0]
    #else:
    #    p_hit[0] = COSALPHA*_d_max
    #    p_hit[1] = SINALPHA*_d_max
    #    p_hit[2] = inf
    #return p_hit





