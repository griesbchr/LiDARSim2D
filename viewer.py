import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import PathPatch, Polygon, Rectangle, Circle
from matplotlib.path import Path
from matplotlib.widgets import Slider

import coordTrans


def findplotlim(scene, ego_x_y_th, egocar, scenesight, sidebuffer = 0.4 ,scaling=(1,0.75)):
    if scenesight:
        xmin = scene.x_min if egocar.x_init > scene.x_min else egocar.x_init
        xmax = scene.x_max if egocar.x_init < scene.x_max else egocar.x_init
        ymin = scene.y_min if egocar.y_init > scene.y_min else egocar.y_init
        ymax = scene.y_max if egocar.y_init < scene.y_max else egocar.y_init
    else:
        xmin = ego_x_y_th[:,0].min() if ego_x_y_th[:,0].min() < scene.x_min else scene.x_min
        xmax = ego_x_y_th[:,0].max() if ego_x_y_th[:,0].max() > scene.x_max else scene.x_max
        ymin = ego_x_y_th[:,1].min() if ego_x_y_th[:,1].min() < scene.y_min else scene.y_min
        ymax = ego_x_y_th[:,1].max() if ego_x_y_th[:,1].max() > scene.y_max else scene.y_max

    #Scaling = x to y is 1 to 1
    delx = (xmax-xmin)
    dely = (ymax-ymin)
    if delx < dely: #x smaller than y, add difference to xlim
        xmax = xmax + (dely-delx)/2
        xmin = xmin - (dely-delx)/2
    else: #y smaller than x, add difference to xlim
        ymax = ymax + (delx-dely)/2
        ymin = ymin - (delx-dely)/2

    #Add sidebuffer
    delx = (xmax-xmin)*sidebuffer
    dely = (ymax-ymin)*sidebuffer
    xmax += delx/2
    xmin -= delx/2
    ymax += dely/2
    ymin -= dely/2

    #Apply scaling
    xmax *= scaling[0]
    xmin *= scaling[0]
    ymax *= scaling[1]
    ymin *= scaling[1]

    return xmin, xmax, ymin, ymax

def sceneViewer(ego_x_y_th, target_x_y_th_list, pointcloud, drawmode, scene, egocar, targetcarlist, T_STEP, scenesight=True):

    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111)
    plt.subplots_adjust(left=0.15, bottom=0.35)  #Adding Space for sliders
    xmin, xmax, ymin, ymax = findplotlim(scene, ego_x_y_th, egocar, scenesight)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    plt.grid()
    ax.set_title("Sceneviewer with lidar "+ drawmode)
    ax.set_axisbelow(True)

    for object in scene.static_objects:
        polygon = Polygon(object.vertices, alpha = 0.5)
        ax.add_patch(polygon)

    #Drawing future drivepath in dotted line
    if drawmode == 'model0' or drawmode == 'model1' or drawmode == 'model2' or drawmode == 'model3':
        ax.scatter(ego_x_y_th[:,0], ego_x_y_th[:,1], marker=".", alpha=0.5, linewidths=0.05, color="grey")
    if drawmode == 'realsensor':
        ax.scatter(ego_x_y_th[::egocar._n_rays,0], ego_x_y_th[::egocar._n_rays,1], marker=".", alpha=0.5, linewidths=0.05, color="grey")

    #Initializing past drivepath
    drivepath = Path(ego_x_y_th[:1,0:2])
    drivepatch = PathPatch(drivepath, linewidth=2, color='grey')
    ax.add_patch(drivepatch)

    #Egocar, Reference position for Rectangle() is lower left corner
    egocarlines = Rectangle(egocar.get_lower_left_coords(ego_x_y_th[0]), egocar._L, egocar._width, fill=False,angle=ego_x_y_th[0][2])
    ax.add_patch(egocarlines)

    #Egocar, Reference position for Rectangle() is lower left corner
    if len(targetcarlist) > 0:
        targetcarlines_list = []
        for carindex, targetcar in enumerate(targetcarlist):
            target_x_y_th = target_x_y_th_list[carindex]
            targetcarlines_list.append(Rectangle(targetcar.get_lower_left_coords(target_x_y_th[0]), targetcar._L, targetcar._width, fill=False,angle=target_x_y_th[0][2]))
            ax.add_patch(targetcarlines_list[carindex])

    #Drawing lidar as blue dot
    lidar_xy = coordTrans.vehicle2globalNoJit(np.array([(0, 0)]), egocar._lidarmountx, egocar._lidarmounty, ego_x_y_th[0, None, :])
    lidarposition = ax.scatter(lidar_xy[0][0], lidar_xy[0][1], marker="o")


    #Initializing hitpoings as orange cross
    turnhitmarker = ax.scatter(pointcloud[0,:,0],pointcloud[0,:,1], marker="x", color="orange")
    turnhitmarker._visible = False
    turnhitmarker._linewidths[0] = 0.5      #Changing linewidth of hitmarker line
    turnhitmarker._factor = 0.75             #Change factor by which size of hitmarker is scales

    # Current lidar ray
    if drawmode == 'realsensor':
        if egocar._fov != 360:
            alpha = egocar.alpha_min + egocar._alpha_inc_value  if egocar._counterclockwise else egocar.alpha_max - egocar._alpha_inc_value
        else:
            alpha = egocar.alpha_init if egocar._counterclockwise else egocar.alpha_init
        alpha += ego_x_y_th[0][2]   #adding rotation of car
        alpha_rad = np.deg2rad(alpha)
        dist = egocar._d_max if np.isinf(pointcloud[0, 0, 2]) else pointcloud[0, 0, 2]
        hit_x = lidar_xy[0][0] + dist * np.cos(alpha_rad)
        hit_y = lidar_xy[0][1] + dist * np.sin(alpha_rad)
        ray, = ax.plot([lidar_xy[0][0],hit_x], [lidar_xy[0][1], hit_y],color='b')

        currenthitmarker = ax.scatter(hit_x, hit_y, marker="x", color="b")
        currenthitmarker._visible = False if np.isinf(pointcloud[0, 0, 2]) else True


    #Drawing circle in indicating the lidars maximum range
    lidarcircle = Circle((lidar_xy[0][0], lidar_xy[0][1]), egocar._d_max, fill=False, edgecolor='orange', alpha = 0.3)
    ax.add_patch(lidarcircle)

    axSlider = plt.axes([0.15, 0.2, 0.75,
                         0.05])  # Creating axes area, (0.1, 0.2) are lower left corner, (0.8, 0.05) are _width an hight (1 is maybe _width and hight of figure)
    slider = Slider(ax=axSlider,
                    label='Time Slider',
                    valmin=0,
                    valstep=1,
                    valmax=ego_x_y_th.shape[0]-1,  # .shape gives size and not the max index, max index is size -1
                    valinit=0)

    def update_val(val):
        sliderval = int(slider.val)
        ego_xy_ll = egocar.get_lower_left_coords(ego_x_y_th[sliderval])
        egocarlines.set_xy((ego_xy_ll))
        egocarlines.angle = ego_x_y_th[sliderval][2] if ego_x_y_th[sliderval][2] < 0 else ego_x_y_th[sliderval][2]+360

        if len(targetcarlist) > 0:
            for carindex, targetcarlines in enumerate(targetcarlines_list):
                target_x_y_th = target_x_y_th_list[carindex]
                target_xy_ll = targetcar.get_lower_left_coords(target_x_y_th[sliderval])
                targetcarlines.set_xy((target_xy_ll))
                targetcarlines.angle = target_x_y_th[sliderval][2] if target_x_y_th[sliderval][2] < 0 else target_x_y_th[sliderval][2]+360

        if drawmode == 'model0' or drawmode == 'model1' or drawmode == 'model2' or drawmode == 'model3':
            if sliderval == 0:
                turnhitmarker._visible = False
            else:
                reduced_pc = pointcloud[sliderval,:,0:2][~np.isinf(pointcloud[sliderval,:,2])]
                turnhitmarker._offsets = reduced_pc
                turnhitmarker._visible = True
        if drawmode == 'realsensor':
            if sliderval // egocar._n_rays == 0:        #// is int division, / would be float division
                turnhitmarker._visible = False
            else:
                reduced_pc = pointcloud[(sliderval//egocar._n_rays)-1,:,0:2][~np.isinf(pointcloud[(sliderval//egocar._n_rays)-1,:,2])]  #-1 after )?
                turnhitmarker._offsets = reduced_pc
                turnhitmarker._visible = True
        lidar_xy = coordTrans.vehicle2globalNoJit(np.array([(0, 0)]), egocar._lidarmountx, egocar._lidarmounty, ego_x_y_th[sliderval, None, :])
        lidarposition._offsets = lidar_xy
        lidarcircle._center = (lidar_xy[0][0],lidar_xy[0][1])

        if drawmode == 'realsensor':
            PC_iter_x = pointcloud[:, :,0].flat  # creates an iterator over the PC that allows direct indexing with sliderval
            PC_iter_y = pointcloud[:, :,1].flat  # creates an iterator over the PC that allows direct indexing with sliderval
            PC_iter_d = pointcloud[:, :,2].flat  # creates an iterator over the PC that allows direct indexing with sliderval

            if egocar._fov != 360:
                counterclockwise = (not egocar._counterclockwise) if ((sliderval//egocar._n_rays)%2) else egocar._counterclockwise
                if counterclockwise:
                    alpha = egocar.alpha_min + egocar._alpha_inc * (sliderval % egocar._n_rays)
                else:
                    alpha = egocar.alpha_max - egocar._alpha_inc * (sliderval % egocar._n_rays)

            else:   #fov == 360
                counterclockwise = egocar._counterclockwise
                if counterclockwise:
                    alpha = egocar.alpha_init + egocar._alpha_inc * (sliderval % egocar._n_rays)
                else:
                    alpha = egocar.alpha_init + egocar._fov - egocar._alpha_inc * (sliderval % egocar._n_rays)
            alpha += ego_x_y_th[sliderval][2]   #adding rotation of car
            alpha_rad = np.deg2rad(alpha)
            dist = egocar._d_max if np.isinf(PC_iter_d[sliderval]) else PC_iter_d[sliderval]
            hit_x = lidar_xy[0][0] + dist*np.cos(alpha_rad)
            hit_y = lidar_xy[0][1] + dist*np.sin(alpha_rad)
            ray.set_data([lidar_xy[0][0], hit_x],[lidar_xy[0][1], hit_y])

            currenthitmarker._offsets = [[hit_x, hit_y]]
            currenthitmarker._visible = False if np.isinf(PC_iter_d[sliderval]) else True

            print("t = ",val*T_STEP)
            if drawmode == 'realsensor':
                print("turn = ", val//egocar._n_rays)

    slider.on_changed(update_val)
    plt.show()

