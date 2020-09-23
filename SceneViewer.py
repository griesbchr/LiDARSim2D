import sys      #needed for access to command line
import time
import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QLabel, QWidget, QPushButton, QSizePolicy ,QShortcut
from PyQt5.QtCore import Qt
import PyQt5.uic as uic

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
#from matplotlib.widgets import Slider, Button

from matplotlib.path import Path
from matplotlib.patches import PathPatch, Polygon, Rectangle, Circle

print("Importing packages... ")
import data
import coordTrans
import scenarios
from getPointcloud import getPointCloudRealSensor as getPointCloudRealSensor
#from getPointcloud import getPointCloudModel0_rt_scenario as getPointCloudModel0
#from getPointcloud import getPointCloudModel1_rt_scenario as getPointCloudModel1
#from getPointcloud import getPointCloudModel2_rt_scenario as getPointCloudModel2
#from getPointcloud import getPointCloudModel3_rt_scenario as getPointCloudModel3

#from getPointcloud import getPointCloudRealSensor_rt as getPointCloudRealSensor_rt
from getPointcloud import getPointCloudModel0_rt as getPointCloudModel0_rt
from getPointcloud import getPointCloudModel1_rt as getPointCloudModel1_rt
from getPointcloud import getPointCloudModel2_rt as getPointCloudModel2_rt
from getPointcloud import getPointCloudModel3_rt as getPointCloudModel3_rt

#from getPointcloud import getPointCloudModel1_rt_scenario_numba as getPointCloudModel1_numba

#print("Importing precompiled functions...")
#import numba_cc
print("Packages imported")

#Fundamentales Problem: Eventloop "stops" at function that is currently executed.
#That means if one function takes 1s to calculate something, the GUI freezes for 1s because the
#eventloop is still at that function. Can be solved with multithreading

class MainWindow(QtWidgets.QMainWindow):
    '''
    Class that contains all of the GUIs functionality
    '''
    def __init__(self, *args, **kwargs):
        '''
        Initializes GUI
        :param args: from commandline
        :param kwargs: from commandline
        '''
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load UI from QTDesigner file
        self.ui = uic.loadUi("sceneViewer.ui", self)

        #Initialize Plotwindow + toolbar
        height = 10 #inches
        width = 10 #inches
        self.canvas = MplCanvas(height=height, width=width,dpi=100)

        toolbar = NavigationToolbar(self.canvas, self)
        self.ui.gridLayout_3.addWidget(toolbar)                #adds Toolbar to layout
        self.ui.gridLayout_3.addWidget(self.canvas)                #adds MplCaravan to layout

        #Add Options to ComboBox according to getScenasioNames function
        for index, name in enumerate(scenarios.getScenarioNames()):
            self.ui.comboBox_scenarios1.insertItem(index, name)
            self.ui.comboBox_scenarios2.insertItem(index, name)

        #Adding flags and initializing values
        self.current_tstep = 0
        self.current_turn = 0
        self.prev_tstep = 0
        self.prev_turn = 0
        self.scenarioloaded_flag = 0
        self.scenariosetup_flag = 0
        self.execerror_flag = 0
        self.scenatioloadingerror_flag = 0
        self.scenario = None
        self.realsensor = None
        self.zoom = 1.5

        #Connecting Signals to functions, if botton is clicked -> function is called
        self.ui.tSlider.valueChanged.connect(self.update_tslider)
        self.ui.pushButton_tstepUp.clicked.connect(self.update_tstepUp)
        self.ui.pushButton_tstepDown.clicked.connect(self.update_tstepDown)
        self.ui.pushButton_tbigUp.clicked.connect(self.update_tBigstepUp)
        self.ui.pushButton_tbigDown.clicked.connect(self.update_tBigstepDown)
        self.ui.pushButton_execute.clicked.connect(self.execute_Sim)
        self.ui.pushButton_execute.pressed.connect(self.update_statusbar)
        self.ui.pushButton_plotstream.clicked.connect(self.execute_plotstream)
        self.ui.tSlider.valueChanged.connect(self.update_plot)
        self.ui.pushButton_zoomin.clicked.connect(self.zoomin)
        self.ui.pushButton_zoomout.clicked.connect(self.zoomout)

        #Setup Shortcuts
        self.shortcut_right = QShortcut(QKeySequence("Right"), self)
        self.shortcut_left = QShortcut(QKeySequence("Left"), self)
        self.shortcut_up = QShortcut(QKeySequence("Up"), self)
        self.shortcut_down = QShortcut(QKeySequence("Down"), self)
        self.shortcut_bigright= QShortcut(QKeySequence("Ctrl+Right"), self)
        self.shortcut_bigleft = QShortcut(QKeySequence("Ctrl+Left"), self)
        self.shortcut_bigup = QShortcut(QKeySequence("Ctrl+Up"), self)
        self.shortcut_bigdown = QShortcut(QKeySequence("Ctrl+Down"), self)

        self.shortcut_right.activated.connect(self.update_tstepUp)
        self.shortcut_left.activated.connect(self.update_tstepDown)
        self.shortcut_up.activated.connect(self.update_tPgstepUp)
        self.shortcut_down.activated.connect(self.update_tPgstepDown)
        self.shortcut_bigright.activated.connect(self.update_tBigstepUp)
        self.shortcut_bigleft.activated.connect(self.update_tBigstepDown)
        self.shortcut_bigup.activated.connect(self.update_tstepmax)
        self.shortcut_bigdown.activated.connect(self.update_tstepmin)

        self.show()

    def update_statusbar(self):
        '''
        Extra function to update statusbar because eventloop doesnt allow two statusbar updates in one function
        :return:
        '''
        self.ui.statusbar.showMessage("Executing...")       #wont be shown because GUI gets updated AFTER function is finnished!

    def execute_plotstream(self):
        #Init Scene with data from stream
        #Update plot with data from stream
        # resetting error flags
        self.execerror_flag = 0
        self.scenatioloadingerror_flag = 0
        self.scenarioloaded_flag = 0
        self.scenariosetup_flag = 0
        self.timestamp = 0
        self.FPS=[1]

        # Clearing plot if scenario is already loaded
        if not (self.scenario is None):
            self.canvas.ax.cla()
            self.current_tstep = 0
            self.prev_tstep = 0
            self.current_turn = 0
            self.prev_turn = 0
            self.realsensor = None
            data.TargetCar.clearTargetList()  # needed because targetcar Class always appends cars to the targetlist (classvariable)


        self.getPlotStreamParams()

        func_name = "get"+ self.ui.comboBox_scenarios2.currentText()
        getScenario = getattr(scenarios, func_name) #Scenarios is the filename, funcname the name of function
        self.scenario = getScenario(self.TSIM, realsensor = False)

        self.setupDynamicPlot()
        self.ui.statusbar.showMessage("Setup completed")
        self.tabWidget_simcontrol.setCurrentIndex(0)
        self.scenariosetup_flag = 1

        # Adding event signal loop
        #dynamic canvas for stream! -> function call every XXms
        self._timer = self.canvas.new_timer(self.signaltime, [(self._update_canvas, (), {})])
        self.ui.pushButton_stopstream_2.clicked.connect(self._timer.stop)
        self.ui.pushButton_stopstream.clicked.connect(self._timer.stop)
        self.ui.pushButton_resumestream.clicked.connect(self._timer.start)

        self._timer.start()

    def getPlotStreamParams(self):
        '''
        Fetch simulation parameters from GUI and data files
        :return:
        '''
        #TSIM
        try:
            self.TSIM = float(self.ui.lineEdit_TSIMStream.text().replace(',', '.'))
        except:
            self.execerror_flag = 1

        self.signaltime = int(self.ui.lineEdit_signaltime.text())

        #Simmode
        if self.ui.radioButton_model0_stream.isChecked():
            self.simmode = 0
        elif self.ui.radioButton_model1_stream.isChecked():
            self.simmode = 1
        elif self.ui.radioButton_model2_stream.isChecked():
            self.simmode = 2
        elif self.ui.radioButton_model3_stream.isChecked():
            self.simmode = 3

        self.realsensor = 0
        self.scenesight = 0
        self.enable_numba = 0

    def setupDynamicPlot(self):
        '''
        Setup for canvas and slider.
        For RealSensor: 1 pagestep is one sensor rotation
        :return:
        '''

        # Setup Cannvas
        self.canvas.ax.set_aspect("equal")
        self.centerplotlim(zoom=self.zoom)
        self.canvas.ax.set_xlim(self.scenario.xmin, self.scenario.xmax)
        self.canvas.ax.set_ylim(self.scenario.ymin, self.scenario.ymax)
        self.canvas.ax.grid()

        self.canvas.ax.set_title("Sceneviewer with lidar model " + str(self.simmode))
        self.canvas.ax.set_axisbelow(True)

        self.setupScene()
        self.setupHitmarkers()


    def _update_canvas(self):
        #print("index = ", self.current_tstep)

        self.getPointCloud()
        self.update_scene()
        self.updateModelHitmarkers()
        if self.realsensor:
            self.updateRealsensor()
        self.canvas.ax.legend(framealpha = 0, loc="upper left", frameon=False)

        self.centerplotlim(zoom=self.zoom)
        self.canvas.ax.set_xlim(self.scenario.xmin, self.scenario.xmax)
        self.canvas.ax.set_ylim(self.scenario.ymin, self.scenario.ymax)
        self.canvas.draw()

        self.current_tstep+=1
        self.prev_tstep = self.current_tstep-1
        self.FPS.append((10 ** 9)/(time.time_ns() - self.timestamp))

        FPS = round(np.average(self.FPS),3)
        timerate = round(FPS*self.scenario.egocar._turning_time,4)
        if len(self.FPS) > 30:
            self.FPS.pop(0) #removes 1st element
        self.ui.statusbar.showMessage(str(FPS)+"FPS     "+str(1/timerate))
        self.timestamp = time.time_ns()

    def execute_Sim(self):
        '''
        Executes Programm, similar structure to main file
        Creates Scenario instance from GUI and scenarios.py
        :return:
        '''
        #resetting error flags
        self.execerror_flag = 0
        self.scenatioloadingerror_flag = 0
        self.scenarioloaded_flag = 0
        self.scenariosetup_flag = 0

        #Clearing plot if scenario is already loaded
        if not(self.scenario is None):
            self.canvas.ax.cla()
            self.current_tstep = 0
            self.prev_tstep = 0
            self.current_turn = 0
            self.prev_turn = 0
            self.realsensor = None
            data.TargetCar.clearTargetList()   #needed because targetcar Class always appends cars to the targetlist (classvariable)

        self.getSimParams()
        self.getScenario()
        if self.scenatioloadingerror_flag == 1:
            self.ui.statusbar.showMessage("Function "+"get"+ self.ui.comboBox_scenarios1.currentText()+" could not be loaded")
            return
        if self.execerror_flag == 1:
            self.ui.statusbar.showMessage("Incorrect input values!")
            return
        self.scenarioloaded_flag = 1

        self.setupPlot()
        self.setupScene()
        self.setupHitmarkers()
        self.ui.statusbar.showMessage("Setup completed")
        self.tabWidget_simcontrol.setCurrentIndex(0)
        self.scenariosetup_flag = 1

    def getSimParams(self):
        '''
        Fetch simulation parameters from GUI and data files
        :return:
        '''
        #TSIM
        try:
            self.TSIM = float(self.ui.lineEdit_TSIM.text().replace(',', '.'))
        except:
            self.execerror_flag = 1
        #Simmode
        if self.ui.radioButton_model0.isChecked():
            self.simmode = 0
        elif self.ui.radioButton_model1.isChecked():
            self.simmode = 1
        elif self.ui.radioButton_model2.isChecked():
            self.simmode = 2
        elif self.ui.radioButton_model3.isChecked():
            self.simmode = 3

        if self.ui.checkBox_calcnumba.isChecked():
            self.enable_numba = 1
        else:
            self.enable_numba = 0
        #Realsensor
        if self.ui.checkBox_realsensor.isChecked():
            self.realsensor = 1
        else:
            self.realsensor = 0
        #SceneSight
        if self.ui.checkBox_scenesight.isChecked():
            self.scenesight = 1
        else:
            self.scenesight = 0
        #Aspect Ratio
        if self.ui.radioButton_aspectval.isChecked():
            try:
                self.aspectratio = str(float(self.ui.lineEdit_aspect.text().replace(',', '.')))
            except:
                self.execerror_flag = 1
        elif self.ui.radioButton_aspectauto.isChecked():
            self.aspectratio = "auto"
        elif self.ui.radioButton_aspectequal.isChecked():
            self.aspectratio = "equal"

    def getScenario(self):
        #load scenario selected in GUI
        func_name = "get"+ self.ui.comboBox_scenarios1.currentText()
        try:
            getScenario = getattr(scenarios, func_name) #Scenarios is the filename, funcname the name of function
            self.scenario = getScenario(self.TSIM, self.realsensor)
        except:
            self.scenatioloadingerror_flag = 1

    def setupPlot(self):
        '''
        Setup for canvas and slider.
        For RealSensor: 1 pagestep is one sensor rotation
        :return:
        '''
        #Clear Canvas and reset slider
        #self.canvas.ax.clear()    #Clear previously painted things if excute is pressed multiple times
        self.tSlider.setValue(0)
        self.update_tslider()

        #Setup Cannvas
        self.canvas.ax.set_aspect(self.aspectratio)
        self.findplotlim()
        self.canvas.ax.set_xlim(self.scenario.xmin, self.scenario.xmax)
        self.canvas.ax.set_ylim(self.scenario.ymin, self.scenario.ymax)
        self.canvas.ax.grid()
        if self.realsensor:
            self.canvas.ax.set_title("Sceneviewer with lidar model " + str(self.simmode) + " and realsensor")
        else:
            self.canvas.ax.set_title("Sceneviewer with lidar model " + str(self.simmode))
        self.canvas.ax.set_axisbelow(True)

        #Setup Slider
        self.tSlider.setMaximum(len(self.scenario.t)-1) #-1 because 1st idex is 0
        if self.realsensor:
            self.tSlider.setPageStep(self.scenario.egocar._n_rays)
        else:
            self.tSlider.setPageStep(10)

    def setupScene(self):
        '''
        Plots static objects and initializes dynamic targets (ego and target car)
        Changable line sets names and with underscore!
        Changable line sets:
            egocar_: lower left position of egocar, intended to be used with get_lower_left_coords(ego_x_y_th[index])
            lidarposition_._offsets
            lidarcircle_._center
            targetcars_
        :return:
        '''
        #Plotting static objects
        for object in self.scenario.scene.static_objects:
            polygon = Polygon(object.vertices, alpha=0.5)
            self.canvas.ax.add_patch(polygon)

        #Plotting future drive path
        if self.realsensor:
            self.canvas.ax.scatter(self.scenario.ego_x_y_th[:,0][::self.scenario.egocar._n_rays], self.scenario.ego_x_y_th[:,1][::self.scenario.egocar._n_rays], marker=".", alpha=0.5, linewidths=0.05, color="grey")
        else:
            self.canvas.ax.scatter(self.scenario.ego_x_y_th[:,0], self.scenario.ego_x_y_th[:,1], marker=".", alpha=0.5, linewidths=0.05, color="grey")

        # Initializing past drivepath
        #self.drivepath = Path(self.scenario.ego_x_y_th[:1, 0:2])
        #self.drivepatch = PathPatch(self.drivepath, linewidth=2, color='grey')
        #self.canvas.ax.add_patch(self.drivepatch)

        # Egocar, Reference position for Rectangle() is lower left corner
        self.egocar_ = Rectangle(self.scenario.egocar.get_lower_left_coords(self.scenario.ego_x_y_th[0]), self.scenario.egocar._L, self.scenario.egocar._width, fill=False, angle=self.scenario.ego_x_y_th[0][2])
        self.canvas.ax.add_patch(self.egocar_)

        # Targetcars, Reference position for Rectangle() is lower left corner
        if len(self.scenario.targetlist) > 0:
            self.targetcars_ = []
            for carindex, targetcar in enumerate(self.scenario.targetlist):
                target_x_y_th = self.scenario.target_x_y_th[carindex]
                self.targetcars_.append(Rectangle(targetcar.get_lower_left_coords(target_x_y_th[0]), targetcar._L, targetcar._width,fill=False, angle=target_x_y_th[0][2]))
                self.canvas.ax.add_patch(self.targetcars_[carindex])

        # Drawing lidar as blue dot
        self.lidar_xy = coordTrans.vehicle2globalNoJit(np.array([(0, 0)]), self.scenario.egocar._lidarmountx, self.scenario.egocar._lidarmounty, self.scenario.ego_x_y_th[0, None, :])
        self.lidarposition_ = self.canvas.ax.scatter(self.lidar_xy[0][0], self.lidar_xy[0][1], marker="o")

        # Drawing circle in indicating the lidars maximum range
        self.lidarcircle_ = Circle((self.lidar_xy[0][0], self.lidar_xy[0][1]), self.scenario.egocar._d_max, fill=False, edgecolor='orange', alpha=0.3)
        self.canvas.ax.add_patch(self.lidarcircle_)

        self.canvas.draw()  #updates plot

    def setupHitmarkers(self):
        '''
        Sets up hitmarkers calculated by the model for each turn. Visibility is set so false because
        at first are no markers visible. The visible markers are always the markers from the last turn.
        :return:
        '''

        # Initializing hitpoints as orange crosses
        self.modelhitmarker_ = self.canvas.ax.scatter(np.zeros((1,self.scenario.egocar._n_rays)), np.zeros((1,self.scenario.egocar._n_rays)), marker="x", color="orange", label="model hitmarkers")
        self.modelhitmarker_._visible = False
        self.modelhitmarker_._linewidths[0] = 0.5  # Changing linewidth of hitmarker line
        self.modelhitmarker_._factor = 0.75  # Change factor by which size of hitmarker is scales

        if self.realsensor:
            self.setupRealsensor()

        self.canvas.ax.legend(framealpha = 1, loc="upper left", frameon = False )

    def setupRealsensor(self):
        '''
        Setup the Lidar Ray and the hitmarkers of the realSensor array.
        Changeable parameters:
            self.ray_
            self.currenthitmarker_
        :return:
        '''
        self.getPointCloud_realsensor()
        #alpha = self.scenario.egocar.alpha_min if self.scenario.egocar._counterclockwise else self.scenario.egocar.alpha_max
        alpha = self.scenario.egocar.current_alpha
        alpha += self.scenario.ego_x_y_th[0][2]
        alpha_rad = np.deg2rad(alpha)
        dist = self.scenario.egocar._d_max if np.isinf(self.pointcloud_realsensor[0, 0, 2]) else self.pointcloud_realsensor[0, 0, 2]
        hit_x = self.lidar_xy[0][0] + dist * np.cos(alpha_rad)
        hit_y = self.lidar_xy[0][1] + dist * np.sin(alpha_rad)
        self.ray_, = self.canvas.ax.plot([self.lidar_xy[0][0], hit_x], [self.lidar_xy[0][1], hit_y], color='b')

        self.currenthitmarker_ = self.canvas.ax.scatter(hit_x, hit_y, marker="x", color="red", label="current hitmarker")
        self.currenthitmarker_._visible = False if np.isinf(self.pointcloud_realsensor[0, 0, 2]) else True

        # Setting up real sensor turn hitmarkers
        self.turnhitmarker_ = self.canvas.ax.scatter(np.zeros((1,self.scenario.egocar._n_rays)), np.zeros((1,self.scenario.egocar._n_rays)), marker="x", color="green", label="realsensor hitmarkers")
        self.turnhitmarker_._visible = False
        self.turnhitmarker_._linewidths[0] = 0.5  # Changing linewidth of hitmarker line
        self.turnhitmarker_._factor = 0.75  # Change factor by which size of hitmarker is scales

    def update_plot(self):
        '''
        updates plot
        :return:
        '''
        if self.scenariosetup_flag == 0:   #when resetting plot, time slider is getting resetted. That triggers update_plot function. This flag prevents that
            return

        self.getPointCloud()
        self.update_scene()
        self.updateModelHitmarkers()
        if self.realsensor:
            self.updateRealsensor()
        self.canvas.ax.legend(framealpha = 0, loc="upper left", frameon=False)
        self.canvas.draw()

    def update_scene(self):
        '''
        Updates the line attributes of the moving objects in plot. Changable attribute
        variables are marked with a trailing _. Following variables are updatable:
            egocar_: lower left position of egocar, intended to be used with get_lower_left_coords(ego_x_y_th[index])
            lidarposition_._offsets
            lidarcircle_._center
            targetcars_
        :return:
        '''
        if self.scenariosetup_flag == 0: return    #if slider is changed before scenatio is set up

        #self.current_tstep = self.tSlider.value()

        #Update Ego position
        ego_xy_ll = self.scenario.egocar.get_lower_left_coords(self.scenario.ego_x_y_th[self.current_tstep])
        self.egocar_.set_xy((ego_xy_ll))
        self.egocar_.angle = self.scenario.ego_x_y_th[self.current_tstep][2] if self.scenario.ego_x_y_th[self.current_tstep][2] < 0 else self.scenario.ego_x_y_th[self.current_tstep][2]+360

        #Update Target Positions
        if len(self.scenario.targetlist) > 0:
            for carindex, targetcar_ in enumerate(self.targetcars_):
                target_x_y_th = self.scenario.target_x_y_th[carindex]
                target_xy_ll = self.scenario.targetlist[carindex].get_lower_left_coords(target_x_y_th[self.current_tstep])
                targetcar_.set_xy((target_xy_ll))
                targetcar_.angle = target_x_y_th[self.current_tstep][2] if target_x_y_th[self.current_tstep][2] < 0 else target_x_y_th[self.current_tstep][2]+360

        #Update lidar position (blue dot) and lidar range circle
        self.lidar_xy = coordTrans.vehicle2globalNoJit(np.array([(0, 0)]), self.scenario.egocar._lidarmountx, self.scenario.egocar._lidarmounty,
                                              self.scenario.ego_x_y_th[self.current_tstep, None, :])
        self.lidarposition_._offsets = self.lidar_xy
        self.lidarcircle_._center = (self.lidar_xy[0][0], self.lidar_xy[0][1])

        #self.canvas.draw()   #updates plot

    def updateModelHitmarkers(self):
        '''
        Placing the markers to the according position calculated in pointcloud.
        Pointcloud returns inf for rays that didnt hit anything. These need to be filtered out becuse they
        dont get plotted.
        The hits from the previous sensor rotation are displayed!
        :return:
        '''
        if self.realsensor:
            if self.prev_turn == self.current_turn: return
            current_tstep = self.current_turn * self.scenario.egocar._n_rays
        else:
            current_tstep = self.current_tstep


        if current_tstep == 0:      #No hitmarkers at first turn
            self.modelhitmarker_._visible = False
        else:
            if self.realsensor and self.prev_turn == self.current_turn: return

            reduced_pc = self.pointcloud[:,0:2][~np.isinf(self.pointcloud[:,2])]
            self.modelhitmarker_._offsets = reduced_pc
            self.modelhitmarker_._visible = True

    def updateRealsensor(self):

        PC_iter_x = self.pointcloud_realsensor[:, :,0].flat  # creates an iterator over the PC that allows direct indexing with sliderval
        PC_iter_y = self.pointcloud_realsensor[:, :,1].flat  # creates an iterator over the PC that allows direct indexing with sliderval
        PC_iter_d = self.pointcloud_realsensor[:, :,2].flat  # creates an iterator over the PC that allows direct indexing with sliderval

        #counterclockwise = self.scenario.egocar.counterclockwise
        if self.scenario.egocar._fov == 360:
            counterclockwise = self.scenario.egocar._counterclockwise
            if counterclockwise:
                alpha = self.scenario.egocar.alpha_init + self.scenario.egocar._alpha_inc * (self.current_tstep % self.scenario.egocar._n_rays)
            else:
                alpha = self.scenario.egocar.alpha_init - self.scenario.egocar._alpha_inc * (self.current_tstep % self.scenario.egocar._n_rays)
        else:
            counterclockwise = not(self.scenario.egocar._counterclockwise) if self.current_turn % 2 else self.scenario.egocar._counterclockwise
            if counterclockwise:
                alpha = self.scenario.egocar.alpha_min + self.scenario.egocar._alpha_inc * (self.current_tstep % self.scenario.egocar._n_rays)
            else:
                alpha = self.scenario.egocar.alpha_max - self.scenario.egocar._alpha_inc * (self.current_tstep % self.scenario.egocar._n_rays)

        alpha += self.scenario.ego_x_y_th[self.current_tstep][2]

        alpha_rad = np.deg2rad(alpha)
        dist = self.scenario.egocar._d_max if np.isinf(PC_iter_d[self.current_tstep]) else PC_iter_d[self.current_tstep]
        hit_x = self.lidar_xy[0][0] + dist * np.cos(alpha_rad)
        hit_y = self.lidar_xy[0][1] + dist * np.sin(alpha_rad)
        self.ray_.set_data([self.lidar_xy[0][0], hit_x], [self.lidar_xy[0][1], hit_y])

        self.currenthitmarker_._offsets = [[hit_x, hit_y]]
        self.currenthitmarker_._visible = False if np.isinf(PC_iter_d[self.current_tstep]) else True

        #Turn hitmarkers
        if self.prev_turn == self.current_turn: return
        if self.current_tstep // self.scenario.egocar._n_rays == 0:  # // is int division, / would be float division
            self.turnhitmarker_._visible = False
        else:
            reduced_pc = self.pointcloud_realsensor[(self.current_tstep // self.scenario.egocar._n_rays) - 1, :, 0:2][
                ~np.isinf(self.pointcloud_realsensor[(self.current_tstep // self.scenario.egocar._n_rays) - 1, :, 2])]
            self.turnhitmarker_._offsets = reduced_pc
            self.turnhitmarker_._visible = True

    def getPointCloud_realsensor(self):
        '''
        Get fully calculated pointcould for all the simulation points
        :return:
        '''
        self.pointcloud_realsensor = getPointCloudRealSensor(*self.scenario.getPCargs("realsensor_fullscenario"))

    def getPointCloud(self):
        '''
        calculates pointcloud for given ego_x_y_th and target_x_y_th (packed in scenario)
        :return:
        '''
        if self.realsensor:
            self.prev_turn = self.current_turn  #from last turn before update
            self.current_turn = self.current_tstep // self.scenario.egocar._n_rays

            if self.prev_turn == self.current_turn: return   #dont load now PC if the one from the previous turn is still valid

            current_tstep = self.current_turn * self.scenario.egocar._n_rays
            prev_tstep = current_tstep - self.scenario.egocar._n_rays
        else:
            current_tstep = self.current_tstep
            prev_tstep = self.prev_tstep

        #pack target_x_y_th for current_tstep and prev_tstep(for interpolation) in new list
        if len(self.scenario.targetlist) > 0:
            target_x_y_th_list = [i[[current_tstep]] for i in self.scenario.target_x_y_th]
            target_x_y_th = np.stack(target_x_y_th_list)    #(converts to (#cars, 1, 3) numpy array, 1 is timesteps, 3 is coords
            if self.simmode != 0:
                target_x_y_th_list_prev = [i[[prev_tstep]] for i in self.scenario.target_x_y_th]
                target_x_y_th_prev = np.stack(target_x_y_th_list_prev)    #(converts to (#cars, 1, 3) numpy array, 1 is timesteps, 3 is coords
        else:
            target_x_y_th = None
            target_x_y_th_prev = None

        if self.enable_numba:
            #ego_x_y_th = self.scenario.ego_x_y_th[current_tstep]
            #ego_x_y_th_prev = self.scenario.ego_x_y_th[prev_tstep]
            print("not implemented")
            return

        if self.simmode == 0:
            #self.pointcloud = getPointCloudModel0(self.scenario.egocar, self.scenario.targetlist, self.scenario.scene, self.scenario.ego_x_y_th[current_tstep], target_x_y_th)
            self.pointcloud = getPointCloudModel0_rt(*self.scenario.getPCargs("model0", current_tstep))
        elif self.simmode == 1:
            #self.pointcloud = getPointCloudModel1(self.scenario.egocar, self.scenario.targetlist, self.scenario.scene, self.scenario.ego_x_y_th[current_tstep], target_x_y_th,self.scenario.ego_x_y_th[prev_tstep], target_x_y_th_prev)
            self.pointcloud = getPointCloudModel1_rt(*self.scenario.getPCargs("model1", current_tstep, prev_index=prev_tstep))
        elif self.simmode == 2:
            #self.pointcloud = getPointCloudModel2(self.scenario.egocar, self.scenario.targetlist, self.scenario.scene, self.scenario.ego_x_y_th[current_tstep], target_x_y_th,self.scenario.ego_x_y_th[prev_tstep], target_x_y_th_prev)
            self.pointcloud = getPointCloudModel2_rt(*self.scenario.getPCargs("model2", current_tstep, prev_index=prev_tstep))
        elif self.simmode == 3:
            #self.pointcloud = getPointCloudModel3(self.scenario.egocar, self.scenario.targetlist, self.scenario.scene, self.scenario.ego_x_y_th[current_tstep], target_x_y_th,self.scenario.ego_x_y_th[prev_tstep], target_x_y_th_prev)
            self.pointcloud = getPointCloudModel3_rt(*self.scenario.getPCargs("model3", current_tstep, prev_index=prev_tstep))


    def centerplotlim(self,spacing=0.08, scaling=(1,0.5), zoom=0.5):
        '''
        Calculate plot limits so tha ego car is in the center of the plot.
        The whole lidar range is visible
        sidebuffer is pedding at the sides
        :return:
        '''
        #spacing = 0
        #scaling=(1,1)
        xmax = self.scenario.ego_x_y_th[self.current_tstep][0] + self.scenario.egocar._d_max*(spacing+1)*0.75*zoom
        xmin = self.scenario.ego_x_y_th[self.current_tstep][0] - self.scenario.egocar._d_max*(spacing+1)*0.75*zoom
        ymax = self.scenario.ego_x_y_th[self.current_tstep][1] + self.scenario.egocar._d_max*(spacing+1)*0.75*zoom
        ymin = self.scenario.ego_x_y_th[self.current_tstep][1] - self.scenario.egocar._d_max*(spacing+1)*0.75*zoom

        # Scaling = x to y is 1 to 1
        delx = (xmax - xmin)#/zoom
        dely = (ymax - ymin)#/zoom
        if delx < dely:  # x smaller than y, add difference to xlim
            xmax = xmax + (dely - delx) / 2
            xmin = xmin - (dely - delx) / 2
        else:  # y smaller than x, add difference to xlim
            ymax = ymax + (delx - dely) / 2
            ymin = ymin - (delx - dely) / 2

        # Add sidebuffer
        delx = (xmax - xmin)# * spacing
        dely = (ymax - ymin)# * spacing
        xmax += delx / 2
        xmin -= delx / 2
        ymax += dely / 2
        ymin -= dely / 2

        # Apply scaling
        xmax *= scaling[0]
        xmin *= scaling[0]
        ymax *= scaling[1]
        ymin *= scaling[1]

        self.scenario.xmin = xmin
        self.scenario.ymin = ymin
        self.scenario.xmax = xmax
        self.scenario.ymax = ymax

    def zoomin(self):
        '''
        zooms in if centerplotlim is active
        :return:
        '''
        self.zoom *= 0.75

    def zoomout(self):
        '''
        zooms in if centerplotlim is active
        :return:
        '''
        self.zoom *= 1.25


    def findplotlim(self, spacing=0.08, scaling=(1.5, 0.8), zoom=1):
        if self.scenesight:
            xmin = self.scenario.scene.x_min if self.scenario.egocar.x_init > self.scenario.scene.x_min else self.scenario.egocar.x_init
            xmax = self.scenario.scene.x_max if self.scenario.egocar.x_init < self.scenario.scene.x_max else self.scenario.egocar.x_init
            ymin = self.scenario.scene.y_min if self.scenario.egocar.y_init > self.scenario.scene.y_min else self.scenario.egocar.y_init
            ymax = self.scenario.scene.y_max if self.scenario.egocar.y_init < self.scenario.scene.y_max else self.scenario.egocar.y_init
        else:
            xmin = self.scenario.ego_x_y_th[:, 0].min() if self.scenario.ego_x_y_th[:, 0].min() < self.scenario.scene.x_min else self.scenario.scene.x_min
            xmax = self.scenario.ego_x_y_th[:, 0].max() if self.scenario.ego_x_y_th[:, 0].max() > self.scenario.scene.x_max else self.scenario.scene.x_max
            ymin = self.scenario.ego_x_y_th[:, 1].min() if self.scenario.ego_x_y_th[:, 1].min() < self.scenario.scene.y_min else self.scenario.scene.y_min
            ymax = self.scenario.ego_x_y_th[:, 1].max() if self.scenario.ego_x_y_th[:, 1].max() > self.scenario.scene.y_max else self.scenario.scene.y_max

        # Scaling = x to y is 1 to 1
        delx = (xmax - xmin)
        dely = (ymax - ymin)
        if delx < dely:  # x smaller than y, add difference to xlim
            xmax = xmax + (dely - delx) / 2
            xmin = xmin - (dely - delx) / 2
        else:  # y smaller than x, add difference to xlim
            ymax = ymax + (delx - dely) / 2
            ymin = ymin - (delx - dely) / 2

        # Add sidebuffer
        delx = (xmax - xmin) * spacing
        dely = (ymax - ymin) * spacing
        xmax += delx / 2
        xmin -= delx / 2
        ymax += dely / 2
        ymin -= dely / 2

        # Apply scaling
        xmax *= scaling[0]*zoom
        xmin *= scaling[0]*zoom
        ymax *= scaling[1]*zoom
        ymin *= scaling[1]*zoom

        self.scenario.xmin = xmin
        self.scenario.ymin = ymin
        self.scenario.xmax = xmax
        self.scenario.ymax = ymax

    def update_tslider(self):
        '''
        Updates Time Slider lable and timestep variable
        :return:
        '''
        self.current_tstep = self.ui.tSlider.value()
        self.prev_tstep = self.current_tstep - 1
        self.ui.label_tstep.setNum(self.current_tstep)

    def update_tstepUp(self):
        '''
        Updates parameters after '>' button or right arrow is pressed
        :return:
        '''
        if self.current_tstep < self.tSlider.maximum():
            self.current_tstep += 1    #+1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tstepDown(self):
        '''
        Updates parameters after '<' button or left arrow key is pressed
        :return:
        '''
        if self.current_tstep > self.tSlider.minimum():
            self.current_tstep += -1    #-1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tPgstepDown(self):
        '''
        Updates parameters after up arrow key is pressed
        :return:
        '''

        if self.current_tstep-self.tSlider.pageStep() >= self.tSlider.minimum():
            self.current_tstep += -self.tSlider.pageStep()    #-1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tPgstepUp(self):
        '''
        Updates parameters after up arrow key is pressed
        :return:
        '''

        if self.current_tstep+self.tSlider.pageStep() <= self.tSlider.maximum():
            self.current_tstep += self.tSlider.pageStep()    #-1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tBigstepDown(self):
        '''
        Updates parameters after up - key is pressed
        :return:
        '''
        if self.realsensor:
            bigstepvalue = int(self.scenario.egocar._n_rays/10)
        else:
            bigstepvalue = 3

        if self.current_tstep-bigstepvalue >= self.tSlider.minimum():
            self.current_tstep += -bigstepvalue    #-1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tBigstepUp(self):
        '''
        Updates parameters after up + key is pressed
        :return:
        '''

        if self.realsensor:
            bigstepvalue = int(self.scenario.egocar._n_rays / 10)
        else:
            bigstepvalue = 3

        if self.current_tstep+bigstepvalue <= self.tSlider.maximum():
            self.current_tstep += bigstepvalue  # -1 for next frame
            self.ui.label_tstep.setNum(self.current_tstep)
            self.ui.tSlider.setValue(self.current_tstep)

    def update_tstepmin(self):
        '''
        Updates parameters after ctrl + arrow down key is pressed
        :return:
        '''

        self.current_tstep = self.tSlider.minimum()
        self.ui.label_tstep.setNum(self.current_tstep)
        self.ui.tSlider.setValue(self.current_tstep)

    def update_tstepmax(self):
        '''
        Updates parameters after ctrl + arrow up key is pressed
        :return:
        '''

        self.current_tstep = self.tSlider.maximum()
        self.ui.label_tstep.setNum(self.current_tstep)
        self.ui.tSlider.setValue(self.current_tstep)




class MplCanvas(FigureCanvasQTAgg):
    '''
    Initializes Matplotlibs Figure Object
    '''

    def __init__(self,height, width, dpi, parent=None):
        self.fig = Figure(figsize=(height,width),dpi=dpi, tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        super(MplCanvas, self).__init__(self.fig)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)  #sys.argv is to allow commandline arguments to be passed
    app.setApplicationName("Scene Viewer")
    window = MainWindow()
    sys.exit(app.exec_())     #This starts the event loop within QApplication instance, sys.exit is to print error codes in console
