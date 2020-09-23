import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch, Polygon, Rectangle
from math import fmod

from coordTrans import getTargetcarVertices


class Car():
    '''
    Basic Class that holds data about cars in the scenario.
    '''

    _L = 3  # wheel base [m]
    _foh = 0.5  # front overhang [m]
    _roh = 0.5  # rear overhang [m]
    _width = 2  # car width [m]
    _steeringangle_max = 59.59  # max. steering angle [deg] from matlab program
    _steeringrate_max = 57.3  # max. steering rate [deg/s] from matlab program
    _lowerleft = np.array([(-_roh, -_width / 2)])
    _lowerright = np.array([(_L - _roh, -_width / 2)])
    _upperleft = np.array([(-_roh, _width / 2)])
    _upperright = np.array([(_L - _roh, _width / 2)])

    def init(self, position_init=(0,0,0), velocity_init=13 ):
        self.x_init = position_init[0]
        self.y_init = position_init[1]
        self.theta_init = position_init[2]
        self.velocity_init = velocity_init
        self.velocity_init_x = self.velocity_init * np.cos(np.deg2rad(self.theta_init))
        self.velocity_init_y = self.velocity_init * np.sin(np.deg2rad(self.theta_init))

    def get_lower_left_coords(self, x_y_th):
        angle_rad = np.deg2rad(x_y_th[2])
        angle_rad = np.deg2rad(x_y_th[2])
        m_trans = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                            [np.sin(angle_rad), np.cos(angle_rad)]])
        shift = m_trans @ self._lowerleft.T
        return x_y_th[0]+shift[0], x_y_th[1]+shift[1]

    def draw(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        shift = self.get_lower_right_coords(self.theta_init)
        rectangle = Rectangle((self.x_init+shift[0],self.y_init+shift[1]), self._L, self._width, angle=self.theta_init, fill=False)
        ax.add_patch(rectangle)
        ax.set_xlim(-5, 20)
        ax.set_ylim(-10, 10)
        plt.grid()
        plt.show()

class TargetCar(Car):
    '''
    Target Cars in the scenario. Instances of this automatically get added to the targetlist.
    '''
    targetlist = []

    def __init__(self):
        super().__init__()
        TargetCar.targetlist.append(self)

    def getTargetcarVertices(self, target_x_y_th):
        '''
        :param target_x_y_th:
        :return:
        '''
        vertices = np.zeros((target_x_y_th.shape[0],4,2),dtype=float)
        line_vecs = np.zeros((target_x_y_th.shape[0],4,2),dtype=float)

        angle_rad = np.deg2rad(target_x_y_th[:,2])
        for i in range(target_x_y_th.shape[0]):
            m_trans = np.array([[np.cos(angle_rad[i]), -np.sin(angle_rad[i])],
                                [np.sin(angle_rad[i]), np.cos(angle_rad[i])]])
            shift_ll = m_trans @ self._lowerleft
            shift_lr = m_trans @ self._lowerright
            shift_ul = m_trans @ self._upperleft
            shift_ur = m_trans @ self._upperright
            vertices[i][0]= target_x_y_th[i][0:2]+shift_ll
            vertices[i][1]= target_x_y_th[i][0:2]+shift_lr
            vertices[i][2]= target_x_y_th[i][0:2]+shift_ur
            vertices[i][3]= target_x_y_th[i][0:2]+shift_ul
            line_vecs[i][0] = vertices[i][1]-vertices[i][0]
            line_vecs[i][1] = vertices[i][2]-vertices[i][1]
            line_vecs[i][2] = vertices[i][3]-vertices[i][2]
            line_vecs[i][3] = vertices[i][0]-vertices[i][3]

        #For potential vehicle view option for SceneViewer
        self.x_min = vertices[:,:,0].min()
        self.x_max = vertices[:,:,0].max()
        self.y_min = vertices[:,:,1].min()
        self.y_max = vertices[:,:,1].max()

        return vertices, line_vecs

    @classmethod
    def clearTargetList(cls):
        '''
        Removes all targetcars. Is used when a new scenario is called, while an old scenario was still loaded.
        :return:
        '''
        cls.targetlist.clear()

class EgoCar(Car):
    '''
    EgoCars contain additional information about the lidar
    '''

    def __init__(self):
        super().__init__()
        self.alpha_init = float(0)   #only positiv values!  #lidar angle starting value OR center of sector if fov != 360
        self._fov = 100                                  #always starts scanning in middle of fov, center is selected by alpha_int
        self._d_max = 50.0                                  # in m, has to be float -> add .0 to end
        self._turning_freq = 10                    #Frequency for a FULL turn, even if fov != 360°
        self._alpha_inc_value = float(0.4)                  # in deg, 360/alpha_inc needs to be an int!! #default is 0.4deg
        self._counterclockwise = False                       #init value   #by default mathematical positive turning direction, is starting direction for fov != 360°


        self._lidarmountx = self._L - self._roh             #currently set to x = front,y =  middle of vehicle; position relative to vehicle coordinate system
        self._lidarmounty = 0.0                             #currently set to x = front,y =  middle of vehicle; position relative to vehicle coordinate system

        self._alpha_inc = self._alpha_inc_value     #if self._counterclockwise else -self._alpha_inc_value
        self._turning_time = 1 / self._turning_freq         #Turning time for a FULL turn, even if fov != 360°

        self._n_rays = round(self._fov / self._alpha_inc_value)

        if self._fov == 360:
                # used to keep track of CURRENT state of lidar sensor, they therefore may change during runtime
                self.current_alpha = self.alpha_init
        else:
            self.alpha_max = int(self._fov / 2) + self.alpha_init
            self.alpha_min = -int(self._fov / 2) + self.alpha_init

            # used to keep track of CURRENT state of lidar sensor, they therefore may change during runtime
            if self._counterclockwise:
                #self.alpha_init = self.alpha_min
                self.current_alpha = self.alpha_min
                #self.current_alpha = self.alpha_init
            else:
                #self.alpha_init = self.alpha_max
                self.current_alpha = self.alpha_max
                #self.current_alpha = self.alpha_init

        #used to keep track of CURRENT state of lidar sensor, they therefore may change during runtime
        self.counterclockwise = self._counterclockwise

class StaticTarget():
    '''
    Contains a numpy array of vertices.
    '''

    def __init__(self, vertices):
        '''
        :param vertices: ither a list of Coordinates or an array of size (2,1)
        '''
        self.vertices = np.array(vertices)
        self.number_of_segments = self.vertices.shape[0]

class Scene():
    '''
    Adds multiple StaticTargets to one Scene.
    self.line_vecs: numpy array containing the line vectors of all lines that make up scene (#lines, 2)
    self.vertices: numpy array containing the vertices of all StaticTargets (#lines,2)
    self.x_min: Smallest x coordinate of any object
    self.x_max: Biggest x coordinate of any object
    self.y_min: Smallest y coordinate of any object
    self.y_max: Biggest y coordinate of any object
    '''

    def __init__(self, static_object_list = []):
        '''
        :param static_object_list: list of instances from StaticTarget class
        '''
        self.static_objects = static_object_list
        self.number_of_objects = len(static_object_list)
        if self.number_of_objects == 0:
            self.line_vecs = None
            self.vertices = None
            self.x_min = 0
            self.x_max = 1
            self.y_min = 0
            self.y_max = 1
            return

        self.line_vecs = []
        self.vertices = []
        for obj_count, object in enumerate(static_object_list):    #Loops threw objects in object_list
            self.vertices.append(object.vertices)
            for vertex_idx, vertex in enumerate(object.vertices): #Loops threw vertices (Eckpunkte) of object
                if vertex_idx + 1 < object.number_of_segments:
                    next_vertex = object.vertices[vertex_idx + 1]   #Vertex of next iteration
                else:
                    next_vertex = object.vertices[0]
                self.line_vecs.append(np.array([(next_vertex[0] - vertex[0]),(next_vertex[1] - vertex[1])]))

        self.number_of_segments = len(self.line_vecs)
        self.line_vecs = np.array(self.line_vecs, dtype=float)           #dim is (#line_vecs, 2)
        self.vertices = np.vstack(self.vertices)
        self.vertices = np.array(self.vertices,dtype=float)
        self.x_min = self.vertices[:,0].min()
        self.x_max = self.vertices[:,0].max()
        self.y_min = self.vertices[:,1].min()
        self.y_max = self.vertices[:,1].max()

    def draw(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)

        for object in self.static_objects:
            polygon = Polygon(object.vertices, alpha=0.5)
            ax.add_patch(polygon)

        ax.set_xlim(-5, 50)
        ax.set_ylim(-20, 20)
        plt.grid()
        plt.show()

class Scenario():
    '''
    A scenario contains stationary and moving objects.
        Stationary objects are defined with the Scene class which contains instances of the StaticTarget class.
        Moving objects are the EgoCar and the TargetCars.
    contains:
        t: time vector for scenario
        scene: stationary objects
        ego_x_y_th: contains x,y position and horizontal angle at each timestep defines in time vector
        target_x_y_th: list that contains for each target vehicle:  x,y position and horizontal angle at each timestep defines in time vector
    '''
    def __init__(self,TSIM, realsensor, scene, egocar, w_ego = None, targetlist=[], w_target_list=[]):
        self.TSIM = TSIM
        self.realsensor = realsensor
        self.scene = scene
        self.egocar = egocar
        self.w_ego = w_ego
        self.targetlist = targetlist
        self.w_target_list = w_target_list

        self.getTimeVector()        #calculate t

        self.ego_x_y_th = self.getCarTragectory(egocar, w_ego)    #calculate ego_x_y_th

        self.target_x_y_th = []

        #Creating  w_target_list if wasnt created in scenario function
        if len(targetlist) != 0 and len(w_target_list) == 0:
            w_target_list = [None for i in range(len(TargetCar.targetlist))]

        for index, car in enumerate(targetlist):
            self.target_x_y_th.append(self.getCarTragectory(car, w_target_list[index]))

    def getTimeVector(self):
        '''
        Calculates the time step vector for the simualtion of gives scenario
        :param egocar: instance of egocar class, defined in Data file. Contains information about lidar
        :return:
        '''
        if self.realsensor:
            #self.T_STEP = (1 / (self.egocar._turning_freq * self.egocar._n_rays)) * self.egocar._fov / 360  # one sim step for each ray
            self.T_STEP = (1 / (self.egocar._turning_freq * self.egocar._n_rays))  # one sim step for each ray
            self.t = np.arange(0, self.TSIM, self.T_STEP)
            self.n_turns = int(len(self.t) / self.egocar._n_rays)  # number of lidar rotations
        else:
            self.T_STEP = self.egocar._turning_time  # one sim step for each lidar rotation
            self.t = np.append(np.arange(0, self.TSIM, self.T_STEP),
                               self.TSIM)  # arange function doesnt include last step (T_SIM), append it manually
            self.n_turns = int(len(self.t) - 1)

    def getPCargs(self, model, curr_index = None, prev_index = None):
        '''
        Function that extracts all the needed arguments to calculate Pointcloud and returns them as tuple
        The time index starts at 1 to ensure that there is a previous time step which is needed for
        the interpolation models.
        :return:
        '''


        if model == "realsensor_fullscenario":
            self._n_turns = int(len(self.t) / self.egocar._n_rays)      #only needed for "full information problem"
            target_x_y_th = np.stack(self.target_x_y_th)    # converts list of np arrays to a single (#cars, #timesteps, 3) numpy array (3 are coords)
            #generating corner coordinates for each target vehicle
            lowerleft = np.repeat(self.egocar._lowerleft,len(self.targetlist), axis=0)
            lowerright = np.repeat(self.egocar._lowerright,len(self.targetlist), axis=0)
            upperleft = np.repeat(self.egocar._upperleft,len(self.targetlist), axis=0)
            upperright = np.repeat(self.egocar._upperright,len(self.targetlist), axis=0)
            args =  (self.t, self._n_turns, self.ego_x_y_th, self.scene.vertices, self.scene.line_vecs, self.egocar._n_rays, self.egocar.alpha_init, self.egocar._alpha_inc, self.egocar._d_max, self.egocar._lidarmountx, self.egocar._lidarmounty, target_x_y_th, self.egocar._fov, self.egocar._counterclockwise, lowerleft, lowerright, upperleft, upperright)
            return args

        if len(self.targetlist) > 0:

            target_x_y_th_list = [i[[curr_index]] for i in self.target_x_y_th]  #extracts list of target_x_y_th for current timestep
            target_x_y_th = np.stack(target_x_y_th_list)  # converts list of np arrays to a single (#cars, 1, 3) numpy array (1 is timesteps, 3 are coords)
            lowerleft = np.repeat(self.egocar._lowerleft, len(self.targetlist), axis=0)
            lowerright = np.repeat(self.egocar._lowerright, len(self.targetlist), axis=0)
            upperleft = np.repeat(self.egocar._upperleft, len(self.targetlist), axis=0)
            upperright = np.repeat(self.egocar._upperright, len(self.targetlist), axis=0)
            target_vertices, target_line_vecs = getTargetcarVertices(lowerleft, lowerright, upperleft, upperright, target_x_y_th)

        else:
            target_x_y_th = None
            target_vertices = None
            target_line_vecs = None

        if self.realsensor:
            turn_index = curr_index // self.egocar._n_rays
        else:
            turn_index = curr_index
        if self.egocar._fov != 360:
            counterclockwise = self.egocar._counterclockwise if turn_index%2 else not(self.egocar._counterclockwise)
        else:
            counterclockwise = self.egocar._counterclockwise

        if model == "realsensor":

            ego_x_y_th = self.ego_x_y_th[curr_index]

            PC_update = False
            #updating current lidar position and turning direction
            if curr_index != 0:
                alpha_inc = self.egocar._alpha_inc if self.egocar.counterclockwise else - self.egocar._alpha_inc
                self.egocar.current_alpha +=  alpha_inc

                if self.egocar._fov != 360:
                    if self.egocar.counterclockwise:
                        if round(self.egocar.current_alpha,3) >= self.egocar.alpha_max:
                            self.egocar.counterclockwise = not (self.egocar.counterclockwise)
                        if round(self.egocar.current_alpha,3) == self.egocar.alpha_max - self.egocar._alpha_inc:
                            PC_update = True

                    else:
                        if round(self.egocar.current_alpha,3) <= self.egocar.alpha_min:
                            self.egocar.counterclockwise = not (self.egocar.counterclockwise)
                        if round(self.egocar.current_alpha,3) == self.egocar.alpha_min + self.egocar._alpha_inc:
                            PC_update = True

                else: #_fov == 360
                    if self.egocar.counterclockwise:
                        if round(self.egocar.current_alpha,3) >= self.egocar.alpha_init + 360:
                            self.egocar.current_alpha = self.egocar.alpha_init
                        if round(self.egocar.current_alpha,3) >= self.egocar.alpha_init + 360 - self.egocar._alpha_inc:
                            PC_update = True
                    else:
                        if round(self.egocar.current_alpha,3) <= self.egocar.alpha_init - 360:
                            self.egocar.current_alpha = self.egocar.alpha_init
                        if round(self.egocar.current_alpha,3) <= self.egocar.alpha_init - 360 + self.egocar._alpha_inc:
                            PC_update = True
            args = (ego_x_y_th,
                    self.scene.vertices, self.scene.line_vecs, target_line_vecs, target_vertices,
                    self.egocar._d_max, self.egocar._lidarmountx, self.egocar._lidarmounty,
                    self.egocar.current_alpha, PC_update)
            return args

        if model == "model0":
            if self.egocar._fov != 360:
                self.egocar.alpha_init = self.egocar.alpha_min
            ego_x_y_th = self.ego_x_y_th[curr_index]

            args = (ego_x_y_th,
                    self.scene.vertices, self.scene.line_vecs, target_vertices,target_line_vecs,
                    self.egocar._n_rays, self.egocar.alpha_init, self.egocar._fov,self.egocar._alpha_inc,
                    self.egocar._lidarmountx, self.egocar._lidarmounty, self.egocar._d_max)

        if model == "model1":
            if self.egocar._fov != 360 and turn_index != 0:
                #self.egocar.counterclockwise = not(self.egocar.counterclockwise)
                self.egocar.alpha_init = self.egocar.alpha_min if counterclockwise else self.egocar.alpha_max
            #prep ego interpolation
            ego_x_y_th = self.ego_x_y_th[curr_index]
            if prev_index is not None:
                ego_x_y_th_prev = self.ego_x_y_th[prev_index]
            else:
                ego_x_y_th_prev = self.ego_x_y_th[curr_index - 1]


            args =  (ego_x_y_th, ego_x_y_th_prev,
                     self.scene.vertices, self.scene.line_vecs, target_vertices, target_line_vecs,
                     self.egocar._n_rays, self.egocar.alpha_init, self.egocar._fov,self.egocar._alpha_inc,
                     self.egocar._lidarmountx, self.egocar._lidarmounty,
                     counterclockwise, self.egocar._d_max)
        if model == "model2":
            if self.egocar._fov != 360 and turn_index != 0:
                #self.egocar.counterclockwise = not(self.egocar.counterclockwise)
                self.egocar.alpha_init = self.egocar.alpha_min if counterclockwise else self.egocar.alpha_max
            #prep ego interpolation
            ego_x_y_th = self.ego_x_y_th[curr_index]
            if prev_index is not None:
                ego_x_y_th_prev = self.ego_x_y_th[prev_index]
                #prep target interpolation
                if len(self.targetlist) > 0:
                    target_x_y_th_prev_list = [i[[prev_index]] for i in self.target_x_y_th]  # extracts list of target_x_y_th for current timestep
                    target_x_y_th_prev = np.stack(target_x_y_th_prev_list)  # converts list of np arrays to a single (#cars, 1, 3) numpy array (1 is timesteps, 3 are coords)

            else:
                ego_x_y_th_prev = self.ego_x_y_th[curr_index - 1]
                prev_index = curr_index-1
                #prep target interpolation
            if len(self.targetlist) > 0:
                target_x_y_th_prev_list = [i[[prev_index]] for i in self.target_x_y_th]  # extracts list of target_x_y_th for current timestep
                target_x_y_th_prev = np.stack(target_x_y_th_prev_list)  # converts list of np arrays to a single (#cars, 1, 3) numpy array (1 is timesteps, 3 are coords)
            else:
                target_x_y_th_prev = None
                lowerleft = None
                lowerright = None
                upperleft = None
                upperright = None


            args = (ego_x_y_th, target_x_y_th, ego_x_y_th_prev,target_x_y_th_prev,
                    lowerleft, lowerright, upperleft, upperright,
                    self.scene.vertices, self.scene.line_vecs,
                    self.egocar._n_rays, self.egocar.alpha_init, self.egocar._fov, self.egocar._alpha_inc,
                    self.egocar._lidarmountx, self.egocar._lidarmounty,
                    counterclockwise, self.egocar._d_max)
        if model == "model3":
            if self.egocar._fov != 360 and turn_index != 0:
                #self.egocar.counterclockwise = not(self.egocar.counterclockwise)
                self.egocar.alpha_init = self.egocar.alpha_min if counterclockwise else self.egocar.alpha_max
            #prep ego interpolation
            ego_x_y_th = self.ego_x_y_th[curr_index]
            if prev_index is not None:
                ego_x_y_th_prev = self.ego_x_y_th[prev_index]
                # prep target vertices interpolation
                if len(self.targetlist) > 0:
                    target_x_y_th_prev_list = [i[[prev_index]] for i in
                                               self.target_x_y_th]  # extracts list of target_x_y_th for current timestep
                    target_x_y_th_prev = np.stack(
                        target_x_y_th_prev_list)  # converts list of np arrays to a single (#cars, 1, 3) numpy array (1 is timesteps, 3 are coords)
                    target_vertices_prev, target_line_vecs_prev = getTargetcarVertices(lowerleft, lowerright,
                                                                                       upperleft,
                                                                                       upperright, target_x_y_th_prev)

            else:
                ego_x_y_th_prev = self.ego_x_y_th[curr_index - 1]
                prev_index = curr_index-1

                #prep target vertices interpolation
            if len(self.targetlist) > 0:
                target_x_y_th_prev_list = [i[[prev_index]] for i in self.target_x_y_th]  # extracts list of target_x_y_th for current timestep
                target_x_y_th_prev = np.stack(target_x_y_th_prev_list)  # converts list of np arrays to a single (#cars, 1, 3) numpy array (1 is timesteps, 3 are coords)
                target_vertices_prev, target_line_vecs_prev = getTargetcarVertices(lowerleft, lowerright, upperleft,
                                                                                   upperright, target_x_y_th_prev)
            else:
                target_vertices_prev = None
                target_line_vecs_prev = None

            args = (ego_x_y_th, ego_x_y_th_prev,
                    self.scene.vertices, self.scene.line_vecs,
                    target_vertices,target_line_vecs, target_vertices_prev, target_line_vecs_prev,
                    self.egocar._n_rays, self.egocar.alpha_init, self.egocar._fov,self.egocar._alpha_inc,
                    self.egocar._lidarmountx, self.egocar._lidarmounty,
                    counterclockwise,self.egocar._d_max)

        return args

    def getCarTragectory(self, car, w=None):
        '''
        Function that calculates position and angle of a car for a given omega vector for given times in time vector
        :param car: instance of class car (or egocar or targetcar)
        :param w: omega vector, has to be same length as t vector
        :return: car_x_y_th vector
        '''

        if type(w) == type(None):
            w = np.zeros_like(self.t)
        else:
            # cropping w vector to right size
            if len(self.t) > len(w):
                w = np.append(w, np.zeros(len(self.t) - len(w)))
            elif len(self.t) < len(w):
                self.wtoolong_flag = 1
                w = w[0:len(self.t)]

        car_x_y_th_dot = np.empty((len(self.t), 3))
        car_x_y_th = np.empty((len(self.t), 3))
        a = np.zeros_like(self.t)

        v = self.integrator(a, car.velocity_init)
        phi = self.integrator(w, 0, upperlimit=car._steeringangle_max, lowerlimit=-car._steeringangle_max)

        # calculation of theta at all timesteps
        car_x_y_th_dot[:, 2] = np.tan(np.deg2rad(phi)) / car._L * v
        car_x_y_th[:, 2] = self.integrator(car_x_y_th_dot[:, 2], np.deg2rad(car.theta_init))

        # calculation of x and y for all timesteps
        car_x_y_th_dot[:, 0] = np.cos(car_x_y_th[:, 2]) * v
        car_x_y_th_dot[:, 1] = np.sin(car_x_y_th[:, 2]) * v

        car_x_y_th[:, 0] = self.integrator(car_x_y_th_dot[:, 0], car.x_init)
        car_x_y_th[:, 1] = self.integrator(car_x_y_th_dot[:, 1], car.y_init)
        car_x_y_th[:, 2] = np.rad2deg(car_x_y_th[:, 2])

        # fig = plt.figure()
        # ax = fig.add_subplot(221)
        # plt.plot(t, w)
        # ax.set_title('w over t')
        #
        # ax = fig.add_subplot(222)
        # plt.plot(t, phi)
        # ax.set_title('phi over t')
        #
        # ax = fig.add_subplot(223)
        # plt.plot(t, car_x_y_th_dot[:,2])
        # ax.set_title('theta_dot over t')
        #
        # ax = fig.add_subplot(224)
        # plt.plot(t, car_x_y_th[:,2])
        # ax.set_title('theta over t')
        # plt.tight_layout()
        # plt.show()

        return car_x_y_th

    def integrator(self, vec, init_cond, upperlimit=None, lowerlimit=None):
        '''
        Integrates the vector vec. Numerical Integration using the euler forward method.
        :param t: time vector
        :param T_STEP: Time between two elements of t vector
        :param vec: vector to be integrates
        :param init_cond: starting value of integration
        :param upperlimit: upper limit
        :param lowerlimit: lower limit
        :return:
        '''
        res = np.empty_like(self.t)
        res[0] = init_cond

        # for i,_ in enumerate(res[1::]):
        for i in range(len(res) - 1):  # -1 because res[0] is already set
            res[i + 1] = res[i] + self.T_STEP * vec[i]
            if upperlimit != None or lowerlimit != None:
                if res[i + 1] != None and res[i + 1] > upperlimit:
                    res[i + 1] = upperlimit
                if res[i + 1] != None and res[i + 1] < lowerlimit:
                    res[i + 1] = lowerlimit

        return res



