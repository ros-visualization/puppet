#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE


import rospy, roslib
roslib.load_manifest('puppet_gui')

from python_qt_binding import QtCore, QtGui
from QtCore import QEvent, qFatal, QModelIndex, QObject, QRect, QRegExp, \
    QSignalMapper, Qt, QTimer, Signal
from QtGui import QApplication, QColor, QDialog, QFileDialog, QIcon, \
    QItemSelectionModel, QKeyEvent, QMainWindow, QMessageBox, QPalette, QPixmap, \
    QSplitter, QTableView, QVBoxLayout, QWidget
from StringIO import StringIO
from pr2_controllers_msgs.msg import *
from puppet_gui.CollisionChecker import CollisionChecker
from puppet_gui.DoubleSpinBoxDelegate import DoubleSpinBoxDelegate
from puppet_gui.KontrolSubscriber import KontrolSubscriber
from puppet_gui.PosesDataModel import PosesDataModel
from puppet_gui.Ps3Subscriber import Ps3Subscriber
from puppet_gui.SimpleFormat import SimpleFormat
from puppet_gui.SpeakEasySubscriber import SpeakEasySubscriber
from puppet_gui.SpeechEditDelegate import SpeechEditDelegate
from puppet_gui.actions.ActionSet import ActionSet
from puppet_gui.actions.DefaultAction import DefaultAction
from puppet_gui.actions.Pr2LookAtFace import Pr2LookAtFace
from puppet_gui.actions.Pr2MoveHeadAction import Pr2MoveHeadAction
from puppet_gui.actions.Pr2MoveLeftArmAction import Pr2MoveLeftArmAction
from puppet_gui.actions.Pr2MoveRightArmAction import Pr2MoveRightArmAction
from puppet_gui.actions.Pr2SpeakAction import Pr2SpeakAction
from puppet_gui.toolbox import Toolbox
from puppet_gui.toolbox_gui import ToolboxGUI
from python_qt_binding import QtCore, QtGui, loadUi
from sensor_msgs.msg import CompressedImage, Image, JointState
from subprocess import call
from trajectory_msgs.msg import *
import math
import new
import os
from functools import partial
import pr2_mechanism_msgs.srv._ListControllers
import pr2_mechanism_msgs.srv._LoadController
import pr2_mechanism_msgs.srv._SwitchController

#import puppet_gui.resourcesRepo

import random
import roslib
import rviz
import signal
import std_srvs.srv._Empty
import sys
import tempfile

roslib.load_manifest('python_qt_binding')
roslib.load_manifest('rviz')
# Disable rviz_backdrop in Groovy:
#roslib.load_manifest('rviz_backdrop')
roslib.load_manifest('puppet_gui')
import rospy;


# Weird Pythonpath problem causes this module to not 
# be found in the actions subdir...:

#from puppet_gui.ProgramQueue import ProgramQueue



# Reasons for stopping sequences: user pressed stop button, vs. other reasons:
STOP_BUTTON = 1

app = QApplication(sys.argv)

check_collisions = '--no-collision' not in sys.argv
show_point_clouds = '--with-point-clouds' in sys.argv

main_window = QMainWindow()
ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src/QtCreatorFiles/puppet_gui', 'puppet_gui.ui')
loadUi(ui_file, main_window)

resourcesDirFromNodeDir = os.path.realpath(os.path.join(os.path.dirname(__file__), '../src/puppet_gui/resources'))
main_window.speechPlayButton.setIcon(QIcon(os.path.join(resourcesDirFromNodeDir, 'play.png')));
main_window.speechStopButton.setIcon(QIcon(os.path.join(resourcesDirFromNodeDir, 'stop.png')));
main_window.speechInsertButton.setIcon(QIcon(os.path.join(resourcesDirFromNodeDir, 'recordIcon.png')));

# set icons for tabs
icons = {
    0: 'square.png',
    1: 'triangle.png',
    2: 'circle.png',
    3: 'cross.png',
}
for index, filename in icons.iteritems():
    filename = os.path.join(resourcesDirFromNodeDir, 'tabIcons', filename)
    icon = QIcon(filename)
    if not icon.isNull():
        main_window.PoseList_tabWidget.setTabText(index, '')
        main_window.PoseList_tabWidget.setTabIcon(index, icon)

# hide design-only widgets
#main_window.square_tableWidget.setVisible(False)

sigint_called = False
def sigint_handler(*args):
    global sigint_called
    print('\nsigint_handler()')
    sigint_called = True
    main_window.close()
signal.signal(signal.SIGINT, sigint_handler)
# the timer enables triggering the sigint_handler
timer = QTimer()
timer.start(500)
timer.timeout.connect(lambda: None)

# replace placeholder with rviz visualization panel
index = main_window.robot_view_verticalLayout.indexOf(main_window.robot_view_widget)
stretch = main_window.robot_view_verticalLayout.stretch(index)
main_window.robot_view_verticalLayout.removeWidget(main_window.robot_view_widget)
#****robot_view = rviz.VisualizationPanel()
robot_view = rviz.VisualizationFrame()
robot_view.setSplashPath( "" ) # No splash screen during loading
robot_view.initialize()

view_manager = robot_view.getManager()

reader = rviz.YamlConfigReader()
config = rviz.Config()
reader.readFile( config, "conf/pr2_view.rviz" )
robot_view.load( config )
robot_view.setMenuBar( None )
robot_view.setStatusBar( None )
robot_view.setHideButtonVisibility( False )

# hide rviz display list
robot_view.children()[1].hide()
main_window.robot_view_verticalLayout.insertWidget(index, robot_view, stretch)

# Set up viewports for camera

def switchToView( view_name ):
    view_man = view_manager.getViewManager()
    for i in range( view_man.getNumViews() ):
        if view_man.getViewAt( i ).getName() == view_name:
            view_man.setCurrentFrom( view_man.getViewAt( i ))
            return
    #print( "Did not find view named %s." % view_name )

main_window.front_view_radioButton.clicked.connect(partial(switchToView, "front view"))
main_window.side_view_radioButton.clicked.connect(partial(switchToView, "side view"))
main_window.angled_view_radioButton.clicked.connect(partial(switchToView, "angled view"))

switchToView("angled view")

rospy.init_node('proto_simple', disable_signals=True)
try:
    use_sim_time = rospy.get_param('use_sim_time')
except KeyError:
    use_sim_time = False


# publish image for backdrop

# Note: this is not doing a complete proper ROS image transport
# publisher.  That doesn't support Python.  Instead, I'm publishing
# directly on the /backdrop/compressed topic a
# sensor_msgs/CompressedImage message, with the raw jpeg data just
# loaded into it.  This way my python file doesn't have to decode or
# convert the image data at all.  It does mean when you ask RViz for
# the available image topics, this won't show up, you'll have to type
# "/backdrop" directly into the topic field.  (RViz adds
# "/compressed".)

backdrop_publisher = rospy.Publisher('/backdrop/compressed', CompressedImage, latch=True)
scenes = []
def set_scene(index):
    image = CompressedImage()
    # image.header.frame_id = '/odom_combined'
    # If you ever want to drive the base, you will want to use the line above, or /map or something.
    # Using /base_link means the robot will always stay exactly in the middle of the backdrop.
    image.header.frame_id = '/base_link'
    image.format = 'jpeg'
    try:
        scene = scenes[index]
    except TypeError:
        index = main_window.scene_comboBox.findText(index)
        scene = scenes[index]
    image.data = open(scene).read()
    image.header.stamp = rospy.Time.now()
    backdrop_publisher.publish(image)
path = os.path.join(os.path.dirname(__file__), '..', 'images')
for filename in os.listdir(path):
    (root, ext) = os.path.splitext(filename)
    if ext in ['.jpg']:
        filename = os.path.join(path, filename)
        scenes.append(filename)
scenes.sort()
for index, filename in enumerate(scenes):
    (root, ext) = os.path.splitext(filename)
    text = os.path.basename(root)
    main_window.scene_comboBox.insertItem(index, text)
main_window.scene_comboBox.currentIndexChanged.connect(set_scene)
index = main_window.scene_comboBox.findText('None')
if index != -1:
    main_window.scene_comboBox.setCurrentIndex(index)



def reset_sim():
    print 'reset_sim()'
    service = '/gazebo/reset_simulation'
    rospy.wait_for_service(service)
    reset_sim_service = rospy.ServiceProxy(service, std_srvs.srv._Empty.Empty)
    try:
        reset_sim_service()
    except rospy.ServiceException, e:
        print "Service did not process request: %s" % str(e)

main_window.reset_sim_pushButton.clicked.connect(reset_sim)
if not show_point_clouds:
    main_window.reset_sim_pushButton.setVisible(False)

def autosave_program():
    print 'autosave_program()'
    save_to_filename(os.path.expanduser('~/_autosave.pt'))

def update_sequence_duration():
    print 'update_sequence_duration()'
    model = get_current_model()
    main_window.sequence_duration_label.setText('%.1f' % model.action_sequence().get_duration())

def current_tab_changed(index):
    update_sequence_duration()
main_window.PoseList_tabWidget.currentChanged.connect(current_tab_changed)


class TrajectoryLock():
    def __init__(self, ns, joint_bounds):
        self._ns = ns
        self._subscriber = None
        self._joint_bounds = [float(x) for x in joint_bounds]
        self._publisher = rospy.Publisher('/%s/command' % ns, trajectory_msgs.msg.JointTrajectory)

    def start(self):
        print 'TrajectoryLock.start()'
        if self._subscriber is not None:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber('/%s/state' % self._ns, pr2_controllers_msgs.msg.JointTrajectoryControllerState, self._receive_state)

    def stop(self):
        print 'TrajectoryLock.stop()'
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    def _receive_state(self, msg):
        max_error = max([abs(x) for x in msg.error.positions])

        exceeded = [abs(x) > y for x,y in zip(msg.error.positions, self._joint_bounds)]

        #print "All: %s" % "  ".join(["% .4f" % x for x in msg.error.positions] )

        if any(exceeded):
            #print "Exceeded: %.4f" % max_error

            # Copy our current state into the commanded state
            cmd = trajectory_msgs.msg.JointTrajectory()
            cmd.header.stamp = msg.header.stamp
            cmd.joint_names = msg.joint_names
            cmd.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
            cmd.points[0].time_from_start = rospy.Duration(.125)
            cmd.points[0].positions = msg.actual.positions
            self._publisher.publish(cmd)
        #else:
            #print "Small: %.4f" % max_error

# pass signal across thread boundaries
class JointObserver(QObject):
    current_values_changed = Signal(str)
    _update_current_value_signal = Signal()
    def __init__(self):
        super(JointObserver, self).__init__()
        self.action_set = None
        self._latest_joint_state = None
        self._update_current_value_signal.connect(self._update_current_value)
        self._subscriber = None
        self._trajectory_locks = []
        self._trajectory_locks.append(TrajectoryLock('head_traj_controller_loose', [.08, .08]))
        self._trajectory_locks.append(TrajectoryLock('l_arm_controller_loose', [.02, .02, .02, .02, .02, .06, .06]))
        self._trajectory_locks.append(TrajectoryLock('r_arm_controller_loose', [.02, .02, .02, .02, .02, .06, .06]))

    def start(self):
        print 'JointObserver.start()'
        if self._subscriber is not None:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber('/joint_states', JointState, self._receive_joint_states)
        for trajectory_lock in self._trajectory_locks:
            trajectory_lock.start()

    def stop(self):
        print 'JointObserver.stop()'
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None
        for trajectory_lock in self._trajectory_locks:
            trajectory_lock.stop()

    def _receive_joint_states(self, joint_state_msg):
        self._latest_joint_state = joint_state_msg
        self._update_current_value_signal.emit()

    def _update_current_value(self):
        self.action_set = ActionSet()
        self.action_set.add_action(Pr2MoveHeadAction())
        self.action_set.add_action(Pr2MoveLeftArmAction())
        self.action_set.add_action(Pr2MoveRightArmAction())

        labels = self.action_set.get_labels()

        for label in labels:
            value = self._latest_joint_state.position[self._latest_joint_state.name.index(label)]
            value *= 180.0 / math.pi
            try:
                self.action_set.update_value(label, value)
            except KeyError, e:
                print 'JointObserver._update_current_value() label "%s" not found' % label
                pass

        value = self.action_set.to_string()
        self.current_values_changed.emit(value)

joint_observer = JointObserver()
joint_observer.current_values_changed.connect(main_window.lineEdit.setText)


INPUT_METHOD_SLIDERS = 0
INPUT_METHOD_INTERACTIVE_MARKERS = 1
INPUT_METHOD_ROBOT = 2
main_window.input_method_comboBox.addItem('Sliders', INPUT_METHOD_SLIDERS)
#main_window.input_method_comboBox.addItem('Interactive Markers', INPUT_METHOD_INTERACTIVE_MARKERS)
if not use_sim_time:
    main_window.input_method_comboBox.addItem('Robot', INPUT_METHOD_ROBOT)
def is_in_slider_mode():
    return main_window.input_method_comboBox.currentIndex() == INPUT_METHOD_SLIDERS

def load_controllers(controllers):
    service = '/pr2_controller_manager/list_controllers'
    rospy.wait_for_service(service)
    service_proxy = rospy.ServiceProxy(service, pr2_mechanism_msgs.srv._ListControllers.ListControllers)
    try:
        request = pr2_mechanism_msgs.srv._ListControllers.ListControllersRequest()
        response = service_proxy(request)
        loaded_controllers = response.controllers
    except rospy.ServiceException, e:
        print 'Service did not process request: %s' % str(e)
        return False

    service = '/pr2_controller_manager/load_controller'
    rospy.wait_for_service(service)
    for name in controllers:
        if name in loaded_controllers:
            print 'load_controllers()', 'skip already loaded', name
            continue
        print 'load_controllers()', name
        service_proxy = rospy.ServiceProxy(service, pr2_mechanism_msgs.srv._LoadController.LoadController)
        try:
            request = pr2_mechanism_msgs.srv._LoadController.LoadControllerRequest()
            request.name = name
            response = service_proxy(request)
            if not response.ok:
                return False
        except rospy.ServiceException, e:
            print 'Service did not process request: %s' % str(e)
            return False
    return True

def switch_controllers(start_controllers, stop_controllers):
    print 'switch_controllers()'
    service = '/pr2_controller_manager/switch_controller'
    rospy.wait_for_service(service)
    service_proxy = rospy.ServiceProxy(service, pr2_mechanism_msgs.srv._SwitchController.SwitchController)
    try:
        request = pr2_mechanism_msgs.srv._SwitchController.SwitchControllerRequest()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        response = service_proxy(request)
        if not response.ok:
            return False
    except rospy.ServiceException, e:
        print 'Service did not process request: %s' % str(e)
        return False
    return True

manniquin_controllers_loaded = False
def change_input_method():
    global manniquin_controllers_loaded
    global joint_observer
    index = main_window.input_method_comboBox.currentIndex()
    input_method = main_window.input_method_comboBox.itemData(index)
    print 'change_input_method()', index

    manniquin_controllers = ['r_arm_controller_loose', 'l_arm_controller_loose', 'head_traj_controller_loose']
    standard_controllers = ['r_arm_controller', 'l_arm_controller', 'head_traj_controller']
    if input_method == INPUT_METHOD_ROBOT:
        # setup controllers for manniquin mode
        if not manniquin_controllers_loaded:
            # load controllers once
            print 'change_input_method()', 'load manniquin controllers'
            manniquin_controllers_loaded = load_controllers(manniquin_controllers)
            if not manniquin_controllers_loaded:
                QMessageBox.critical(main_window, main_window.tr('Loading controllers failed'), main_window.tr('Could not load manniquin controllers.'))
        # switch to manniquin controllers
        success = switch_controllers(manniquin_controllers, standard_controllers)
        if success:
            joint_observer.start()
    else:
        # restore standard controllers
        success = switch_controllers(standard_controllers, manniquin_controllers)
        if success:
            joint_observer.stop()
    if not success:
        QMessageBox.critical(main_window, main_window.tr('Switching controllers failed'), main_window.tr('Could not switch controllers.'))

main_window.input_method_comboBox.setCurrentIndex(INPUT_METHOD_SLIDERS)
main_window.input_method_comboBox.currentIndexChanged.connect(change_input_method)
if main_window.input_method_comboBox.count() <= 1:
    main_window.input_method_label.setVisible(False)
    main_window.input_method_comboBox.setVisible(False)
    main_window.input_method_horizontalLayout.parent().removeItem(main_window.input_method_horizontalLayout)


kontrol_subscriber   = KontrolSubscriber()
toolbox_gui = ToolboxGUI.getInstance(main_window, Toolbox())
speakeasy_subscriber = SpeakEasySubscriber(toolbox_gui.commChannel.insertRobotSaysSignal)

#****main_window.toolsPaneFrame.hide()

if check_collisions:
    collision_checker = CollisionChecker()
currently_in_collision = False
default_color = main_window.lineEdit.palette().text().color()

# pass signal across thread boundaries
class Foo(QObject):
    current_values_changed = Signal(str)
    current_duration_changed = Signal(float)
    _update_current_value_signal = Signal()
    _show_input_notification_signal = Signal()
    _hide_input_notification_signal = Signal()
    _show_scene_notification_signal = Signal()
    _hide_scene_notification_signal = Signal()
    _update_color_of_line_edit_signal = Signal()
    def __init__(self):
        super(Foo, self).__init__()
        self._action_set = None
        self._update_current_value_signal.connect(self._update_current_value)
        
        self._scene_notification = QDialog(main_window)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src/QtCreatorFiles/puppet_gui', 'wrong_scene_dialog.ui')
        loadUi(ui_file, self._scene_notification)
        self._scene_notification_timer = QTimer(main_window)
        self._scene_notification_timer.setInterval(5000)
        self._scene_notification_timer.setSingleShot(True)
        self._scene_notification_timer.timeout.connect(self._hide_scene_notification)
        self._scene_notification.finished.connect(self._hide_scene_notification)

        self._input_notification = QDialog(main_window)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src/QtCreatorFiles/puppet_gui/', 'wrong_input_method_dialog.ui')
        loadUi(ui_file, self._input_notification)
        self._input_notification_timer = QTimer(main_window)
        self._input_notification_timer.setInterval(5000)
        self._input_notification_timer.setSingleShot(True)
        self._input_notification_timer.timeout.connect(self._hide_input_method_notification)
        self._input_notification.finished.connect(self._hide_input_method_notification)

        self._show_input_notification_signal.connect(self._show_input_notification)
        self._hide_input_notification_signal.connect(self._hide_input_method_notification)
        self._show_input_notification_signal.connect(self._show_scene_notification)
        self._hide_input_notification_signal.connect(self._hide_scene_notification)
        self._update_color_of_line_edit_signal.connect(self._update_color_of_line_edit)
        
    def _hide_scene_notification(self, result=None):
        self._scene_notification_timer.stop()
        self._scene_notification.hide()

    def _hide_input_method_notification(self, result=None):
        self._input_notification_timer.stop()
        self._input_notification.hide()

    # this method is call by ROS, all UI interaction is delayed via signals
    def update_current_value(self):
        global check_collisions
        global currently_in_collision

        if self._action_set is not None:
            self._action_set.stop()
        self._action_set = kontrol_subscriber.get_action_set()
        if self._action_set is not None:
            self.current_duration_changed.emit(self._action_set.get_duration())

        if not is_in_slider_mode():
            currently_in_collision = False
            self._show_input_notification_signal.emit()
            return
        if self._input_notification_timer.isActive():
            self._hide_input_notification_signal.emit()

        #print('update_current_value()')
        if not kontrol_subscriber.is_valid_action_set():
            self._show_scene_notification_signal.emit()
            return
        if self._scene_notification_timer.isActive():
            self._hide_scene_notification_signal.emit()

        joint_values = kontrol_subscriber.get_joint_values()
        if check_collisions:
            in_collision = collision_checker.is_in_collision(joint_values)
        else:
            in_collision = False

        collision_toggled = (currently_in_collision != in_collision)
        currently_in_collision = in_collision

        if collision_toggled:
            self._update_color_of_line_edit_signal.emit()

        self._update_current_value_signal.emit()

    def _show_input_notification(self):
        # open dialog which closes after some s_hide_scene_notification_hide_scene_notificationeconds or when confirmed manually
        self._input_notification.show()
        self._input_notification_timer.start()

    def _show_scene_notification(self):
        # open dialog which closes after some seconds or when confirmed manually
        self._scene_notification.show()
        self._scene_notification_timer.start()

    def _update_color_of_line_edit(self):
        global currently_in_collision
        palette = main_window.lineEdit.palette()
        if currently_in_collision:
            palette.setColor(QPalette.Text, QColor(255, 0, 0))
        else:
            palette.setColor(QPalette.Text, default_color)
        main_window.lineEdit.setPalette(palette)

    def _update_current_value(self):
        global currently_in_collision
        main_window.append_pushButton.setEnabled(not currently_in_collision)
        main_window.insert_before_pushButton.setEnabled(not currently_in_collision)

        value = self._action_set.to_string()
        self.current_values_changed.emit(value)
        for action in self._action_set._actions:
            action._duration = 0.1 if use_sim_time else 0.5
        self._action_set.execute()
foo = Foo()

kontrol_subscriber.axes_changed.connect(foo.update_current_value)
foo.current_values_changed.connect(main_window.lineEdit.setText)
foo.current_duration_changed.connect(main_window.duration_doubleSpinBox.setValue)

def get_tab_widget(index):
    return main_window.PoseList_tabWidget.widget(index)

def get_table_view(index):
    tab_widget = get_tab_widget(index)
    return tab_widget.findChildren(QTableView, QRegExp('.*_tableView'))[0]

def get_current_tab_index():
    return main_window.PoseList_tabWidget.currentIndex()

def test_clicked(index):
    try:
        row = index.row()
    except:
        row = int(index)
    model = get_current_model()
    print 'execute %d' % row
    model.action_sequence().actions()[row].execute()

# override tab order by ignoring non-editable cells
def custom_focusNextPrevChild(self, next, old_focusNextPrevChild=QTableView.focusNextPrevChild):
    rc = old_focusNextPrevChild(self, next)
    if self.tabKeyNavigation() and self.currentIndex().isValid() and not(self.currentIndex().flags() & Qt.ItemIsEditable):
        # repeat same key press when cell is not editable
        event = QKeyEvent(QEvent.KeyPress, Qt.Key_Tab, Qt.ShiftModifier if next else Qt.NoModifier)
        self.keyPressEvent(event)
        if event.isAccepted():
            return True
    return rc

def create_duration_delegate():
    duration_delegate = DoubleSpinBoxDelegate()
    duration_delegate.setMinimum(main_window.duration_doubleSpinBox.minimum())
    duration_delegate.setMaximum(main_window.duration_doubleSpinBox.maximum())
    duration_delegate.setSuffix(main_window.duration_doubleSpinBox.suffix())
    duration_delegate.setSingleStep(main_window.duration_doubleSpinBox.singleStep())
    return duration_delegate
    
def create_speech_edit_delegate():
    delegate = SpeechEditDelegate()
    return delegate

# override key press event to handle delete key pressed
def custom_keyPressEvent(self, event, old_keyPressEvent=QTableView.keyPressEvent):
    if event.key() == Qt.Key_Delete and event.modifiers() == Qt.NoModifier:
        if delete_selected():
            print 'custom_keyPressEvent()', 'delete row'
            event.accept()
            return
    elif event.key() == Qt.Key_Down and event.modifiers() == Qt.NoModifier:
        select_next_row()
        return
    elif event.key() == Qt.Key_Up and event.modifiers() == Qt.NoModifier:
        select_previous_row()
        return
    return old_keyPressEvent(self, event)

for i in range(main_window.PoseList_tabWidget.count()):
    table_view = get_table_view(i)
    table_view.focusNextPrevChild = new.instancemethod(custom_focusNextPrevChild, table_view, None)
    table_view.keyPressEvent = new.instancemethod(custom_keyPressEvent, table_view, None)

# create models for each table view in the tabs
models = []
# keep reference to delegates to prevent being garbaged
delegates = []
for i in range(main_window.PoseList_tabWidget.count()):
    table_view = get_table_view(i)
    model = PosesDataModel(main_window.edit_model_radioButton.isChecked())
    model.actions_changed.connect(autosave_program)
    model.duration_modified.connect(update_sequence_duration)
    table_view.setModel(model)
    
    duration_delegate = create_duration_delegate()
    table_view.setItemDelegateForColumn(0, duration_delegate)
    delegates.append(duration_delegate)
    
    speech_edit_delegate = create_speech_edit_delegate()
    table_view.setItemDelegateForColumn(1, speech_edit_delegate)
    delegates.append(speech_edit_delegate)
    
    model.add_delegates(table_view)
    table_view.resizeColumnsToContents()
    table_view.clicked.connect(test_clicked)
    table_view.verticalHeader().sectionClicked.connect(test_clicked)
    models.append(model)

def toggle_model_editable():
    for i in range(main_window.PoseList_tabWidget.count()):
        models[i].set_editable(main_window.edit_model_radioButton.isChecked())
        table_view = get_table_view(i)
        table_view.resizeColumnsToContents()

main_window.read_only_model_radioButton.clicked.connect(toggle_model_editable)
main_window.edit_model_radioButton.clicked.connect(toggle_model_editable)

def get_current_model():
    index = get_current_tab_index()
    return models[index]


def get_action_set():
    if is_in_slider_mode():
        action_set = kontrol_subscriber.get_action_set()
        if action_set is not None:
            action_set.set_duration(main_window.duration_doubleSpinBox.value())
    else:
        action_set = joint_observer.action_set.deepcopy()
    return action_set

def append_current():
    if currently_in_collision:
        return

    model = get_current_model()

    action_set = get_action_set()
    if action_set is None:
        return
    model.add_action(action_set)

    table_view = get_table_view(get_current_tab_index())
    rows = len(model.action_sequence().actions())
    table_view.resizeColumnsToContents()
    table_view.selectRow(rows - 1)

main_window.append_pushButton.clicked.connect(append_current)

def select_next_row():
    table_view = get_table_view(get_current_tab_index())
    current_row = get_selected_row()
    if current_row is None:
        current_row = 0
    if current_row >= get_row_count() - 1:
        return
    table_view.selectRow(current_row + 1)
    model = get_current_model() 
    model.action_sequence().actions()[current_row + 1].execute()


def select_previous_row():
    table_view = get_table_view(get_current_tab_index())
    current_row = get_selected_row()
    if current_row is None:
        return
    if current_row == 0 or get_row_count() == 0:
        return
    table_view.selectRow(current_row - 1)
    model = get_current_model()     
    model.action_sequence().actions()[current_row - 1].execute()

main_window.nextLineButton.clicked.connect(select_next_row)
main_window.prevLineButton.clicked.connect(select_previous_row)


def insert_current_before_selected():
    if currently_in_collision:
        return

    row = get_selected_row()
    if row is None:
        append_current()
        return

    model = get_current_model()

    action_set = get_action_set()
    if action_set is None:
        return
    model.add_action(action_set, row)

    table_view = get_table_view(get_current_tab_index())
    table_view.resizeColumnsToContents()
    table_view.selectRow(row + 1)

main_window.insert_before_pushButton.clicked.connect(insert_current_before_selected)


def insert_find_face():
    model = get_current_model()
    action_set = ActionSet()
    action = Pr2LookAtFace()
    action_set.add_action(action)
    action_set.set_duration(main_window.duration_doubleSpinBox.value())
    model.add_action(action_set)

main_window.find_face_pushButton.clicked.connect(insert_find_face)

def toggle_toolbox_visible(checked):
    if checked:
        main_window.toolsPaneFrame.show()
    else:
        main_window.toolsPaneFrame.hide()

main_window.showSoundButton.clicked.connect(toggle_toolbox_visible)
# Set the 'Show Sound' button to reflect whether the tool box
# is open, or not:
if main_window.toolsPaneFrame.isVisible():
    main_window.showSoundButton.setDown(True);
else:
    main_window.showSoundButton.setDown(False);

def bringAccessibleSlidersToTop():
    try:
        call(['wmctrl', '-a', 'Slider Board']);
    except OSError:
        # wmctrl was not installed on the system:
        self.dialogService.showErrorMsg("This button needs wmctrl installed (sudo apt-get install wmctrl).");
        return
    except AttributeError:
        self.dialogService.showErrorMsg("Onscreen sliders seem not to be running. Start them with 'roslaunch puppet_gui accessibleSlider.launch'");
        return;
        
        
main_window.accessibilityButton.clicked.connect(bringAccessibleSlidersToTop);

#@Slot(str,str,str)
def insert_speak(whatToSay, voiceToUse, ttsEngine):
    model = get_current_model()
    action_set = ActionSet()
    shouldBlock = main_window.blockingSpeechCheckbox.isChecked()
    try:
        action = Pr2SpeakAction(Toolbox.getInstance(), 
                                whatToSay, 
                                voice=voiceToUse,
                                ttsEngine=ttsEngine,
                                waitForSpeechDone=shouldBlock) 
    except ValueError as e:
        self.dialogService.showErrorMsg(str(e));
        return;
    action_set.add_action(action)
    model.add_action(action_set)

toolbox_gui.commChannel.insertRobotSaysSignal.connect(insert_speak);


def delete_selected():
    row = get_selected_row()
    if row is None:
        return False

    model = get_current_model()
    model.remove_action(row)

    table_view = get_table_view(get_current_tab_index())
    rows = len(model.action_sequence().actions())
    if row >= rows:
        row = row - 1
    table_view.resizeColumnsToContents()
    table_view.selectRow(row)
    return True

main_window.delete_selected_pushButton.clicked.connect(delete_selected)


def clone_selected():
    row = get_selected_row()
    if row is None:
        return

    model = get_current_model()
    action = model.action_sequence().actions()[row]
    clone = action.deepcopy()
    model.add_action(clone, row + 1)

main_window.clone_selected_pushButton.clicked.connect(clone_selected)


def get_row_count():
    model = get_current_model()
    return model.rowCount()
def get_selected_row():
    table_view = get_table_view(get_current_tab_index())
    selection_model = table_view.selectionModel()
    if selection_model.hasSelection():
        row = selection_model.selectedRows()[0].row()
        #print 'get_selected_row() %d' % row
        return row
    #print 'get_selected_row() None'
    return None
def execute_selected():
    row = get_selected_row()
    if row is not None:
        model = get_current_model()
        print 'execute %d' % row
        model.action_sequence().actions()[row].execute()


def set_selected_row(row):
    #print 'set_selected_row(%d)' % row
    rows = get_row_count()
    if rows == 0:
        return
    if row < 0:
        row = 0
    if row >= rows:
        row = rows - 1
    table_view = get_table_view(get_current_tab_index())
    #print 'set_selected_row() row %d' % row
    table_view.selectRow(row)

class RelaySignalInt(QObject):
    relay_signal = Signal(int)
    def __init__(self):
        super(RelaySignalInt, self).__init__()
    def emit(self, row):
        self.relay_signal.emit(row)

set_selected_row_signal = RelaySignalInt()
set_selected_row_signal.relay_signal.connect(set_selected_row)

running_sequence = None

def set_tab(index):
    if get_current_tab_index() != index:
        main_window.PoseList_tabWidget.setCurrentIndex(index)

def finished_executing_current_sequence():
    global running_sequence
    print 'finished_executing_current_sequence()'
    model = get_current_model()
    action_sequence = model.action_sequence()
    action_sequence.executing_action_signal.disconnect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.disconnect(finished_executing_current_sequence)
    running_sequence = None

def stop_sequence(reason=None):
    global running_sequence
    if running_sequence is None:
        # Speech utterances can run in parallel to motions.
        # So there might be no action registered here, but
        # yet speech actions are running in the background:
        if reason == STOP_BUTTON:
            Pr2SpeakAction.stopAllSpeech();
        print 'stop_sequence() skipped - not running'
        return
    print 'stop_sequence()', running_sequence
    model = models[running_sequence]
    action_sequence = model.action_sequence()
    action_sequence.executing_action_signal.disconnect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.disconnect(finished_executing_current_sequence)
    action_sequence.stop(reason)
    running_sequence = None

def execute_sequence(index, first_action = None):
    global running_sequence
    if running_sequence == -1:
        print 'execute_sequence() skip execute due to running sequence'
        return
    if running_sequence is not None:
        stop_sequence()
    print 'execute_sequence()', index
    running_sequence = index
    set_tab(index)
    model = models[index]
    action_sequence = model.action_sequence()
    action_sequence.executing_action_signal.connect(set_selected_row_signal.emit)
    action_sequence.execute_sequence_finished_signal.connect(finished_executing_current_sequence)
    action_sequence.execute_all(first_action)

def execute_current_sequence():
    execute_sequence(get_current_tab_index())

def execute_current_sequence_from_selection():
    row = get_selected_row()
    if row == get_row_count() - 1:
        row = None
    execute_sequence(get_current_tab_index(), row)

main_window.run_sequence_from_selection_pushButton.clicked.connect(execute_current_sequence_from_selection)


select_row_signal = RelaySignalInt()
select_row_signal.relay_signal.connect(set_selected_row)

collision_notification = QDialog(main_window)
ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src/QtCreatorFiles/puppet_gui', 'collision_dialog.ui')
loadUi(ui_file, collision_notification)
collision_notification_timer = QTimer(main_window)
collision_notification_timer.setInterval(3000)
collision_notification_timer.setSingleShot(True)
def hide_collision_notification(result=None):
    collision_notification_timer.stop()
    collision_notification.hide()
collision_notification_timer.timeout.connect(hide_collision_notification)
collision_notification.finished.connect(hide_collision_notification)

def check_buttons():
    triggered_buttons = kontrol_subscriber.get_triggered_buttons()

    if KontrolSubscriber.previous_button in triggered_buttons:
        if get_selected_row() is not None:
            select_row_signal.emit(get_selected_row() - 1)
        else:
            select_row_signal.emit(0)
    elif KontrolSubscriber.play_button in triggered_buttons:
        execute_selected()
    elif KontrolSubscriber.next_button in triggered_buttons:
        if get_selected_row() is not None:
            select_row_signal.emit(get_selected_row() + 1)
        else:
            select_row_signal.emit(get_row_count() - 1)

    elif KontrolSubscriber.repeat_button in triggered_buttons:
        execute_current_sequence()
    elif KontrolSubscriber.stop_button in triggered_buttons:
        stop_sequence(reason=STOP_BUTTON)
    elif KontrolSubscriber.record_button in triggered_buttons:
        if currently_in_collision:
            # open dialog which closes after some seconds or when confirmed manually
            collision_notification.show()
            collision_notification_timer.start()
        append_current()

    elif KontrolSubscriber.top1_button in triggered_buttons:
        set_tab(0)
    elif KontrolSubscriber.bottom1_button in triggered_buttons:
        set_tab(3)
    elif KontrolSubscriber.top2_button in triggered_buttons:
        set_tab(1)
    elif KontrolSubscriber.bottom2_button in triggered_buttons:
        set_tab(2)

    elif KontrolSubscriber.top6_button in triggered_buttons:
        execute_sequence(0)
    elif KontrolSubscriber.bottom6_button in triggered_buttons:
        execute_sequence(3)
    elif KontrolSubscriber.top7_button in triggered_buttons:
        execute_sequence(1)
    elif KontrolSubscriber.bottom7_button in triggered_buttons:
        execute_sequence(2)

class RelaySignal(QObject):
    relay_signal = Signal()
    def __init__(self):
        super(RelaySignal, self).__init__()
    def emit(self):
        self.relay_signal.emit()

check_buttons_signal = RelaySignal()
check_buttons_signal.relay_signal.connect(check_buttons)
kontrol_subscriber.buttons_changed.connect(check_buttons_signal.emit)


def check_ps3_buttons():
    triggered_buttons = ps3_subscriber.get_triggered_buttons()

    if Ps3Subscriber.select_button in triggered_buttons:
        #sys.exit(0)
        pass
    elif Ps3Subscriber.start_button in triggered_buttons:
        #default_pose()
        pass

    elif Ps3Subscriber.square_button in triggered_buttons:
        execute_sequence(0)
    elif Ps3Subscriber.triangle_button in triggered_buttons:
        execute_sequence(1)
    elif Ps3Subscriber.circle_button in triggered_buttons:
        execute_sequence(2)
    elif Ps3Subscriber.cross_button in triggered_buttons:
        execute_sequence(3)

ps3_subscriber = Ps3Subscriber()
check_ps3_buttons_signal = RelaySignal()
check_ps3_buttons_signal.relay_signal.connect(check_ps3_buttons)
ps3_subscriber.buttons_changed.connect(check_ps3_buttons_signal.emit)


current_name = None

def load_from_file():
    global current_name
    file_name, _ = QFileDialog.getOpenFileName(main_window, main_window.tr('Load program from file'), None, main_window.tr('Puppet Talk (*.pt)'))
    if file_name is None or file_name == '':
        return

    print 'load_from_file', file_name
    handle = open(file_name, 'rb')
    storage = SimpleFormat(handle)

    count = storage.deserialize_data()
    tabs = main_window.PoseList_tabWidget.count()
    for index in range(tabs):
        model = models[index]
        if index < count:
            model.action_sequence().deserialize(storage)
            model.reset()
        else:
            model.remove_all_actions()
    handle.close()

    current_name = os.path.splitext(os.path.basename(file_name))[0]

    update_sequence_duration()

main_window.actionOpen.triggered.connect(load_from_file)


def serialize(storage):
    count = main_window.PoseList_tabWidget.count()
    storage.serialize_data(count)
    for index in range(count):
        model = models[index]
        model.action_sequence().serialize(storage)

def save_to_file():
    file_name, _ = QFileDialog.getSaveFileName(main_window, main_window.tr('Save program to file'), 'example.pt', main_window.tr('Puppet Talk (*.pt)'))
    if file_name is None or file_name == '':
        return False

    save_to_filename(file_name)
    return True

def save_to_filename(file_name):
    global current_name
    print 'save_to_filename', file_name
    handle = open(file_name, 'wb')
    storage = SimpleFormat(handle)
    serialize(storage)
    handle.close()

    current_name = os.path.splitext(os.path.basename(file_name))[0]

main_window.actionSave_As.triggered.connect(save_to_file)


def save_screenshot():
    path = os.path.expanduser('~')
    filename_template = 'robot-%d.png'
    serial = 1
    while True:
        filename = os.path.join(path, filename_template % serial)
        if not os.path.exists(filename):
            break
        serial += 1
    widget = robot_view.children()[0]
    pixmap = QPixmap.grabWindow(widget.winId())
    rc = pixmap.save(filename)
    if rc:
        QMessageBox.information(main_window, main_window.tr('Saved screenshot'), main_window.tr('Screenshot saved in file: %s') % filename)
    else:
        QMessageBox.critical(main_window, main_window.tr('Saving screenshot failed'), main_window.tr('Could not save screenshot.'))

main_window.actionSave_Screenshot.triggered.connect(save_screenshot)


def clear_all():
    global current_name
    confirmed = True
    if current_name is None:
        button = QMessageBox.question(main_window, main_window.tr('Clear all poses'), main_window.tr('Do you really want to delete all poses?'), QMessageBox.No | QMessageBox.Yes)
        confirmed = button == QMessageBox.Yes
    if confirmed:
        print 'clear_all()'
        for index in range(main_window.PoseList_tabWidget.count()):
            model = models[index]
            model.remove_all_actions()
        current_name = None
        set_tab(0)
        main_window.angled_view_radioButton.click()

main_window.actionClear_All.triggered.connect(clear_all)


default_pose = DefaultAction()
default_pose.set_duration(8.0)

def finished_executing_default_pose():
    global running_sequence
    print 'finished_executing_default_pose()'
    running_sequence = None

def execute_default_pose():
    global running_sequence
    if running_sequence is not None:
        stop_sequence()
    print 'execute_default_pose()'
    running_sequence = -1
    default_pose.execute()

default_pose.execute_finished_signal.connect(finished_executing_default_pose)

def add_default_pose():
    model = get_current_model()
    action_set = ActionSet()
    action = DefaultAction()
    action_set.add_action(action)
    action_set.set_duration(main_window.duration_doubleSpinBox.value())
    model.add_action(action_set)

    table_view = get_table_view(get_current_tab_index())
    rows = len(model.action_sequence().actions())
    table_view.resizeColumnsToContents()
    table_view.selectRow(rows - 1)

main_window.actionDefault_Pose.triggered.connect(add_default_pose)


main_window.actionTest_Program.triggered.connect(execute_current_sequence)


queue_default_username = 'admin'
queue_default_password = 'admin'

def queue_dialog():
    dialog = QDialog()
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src/QtCreatorFiles/puppet_gui', 'queue_dialog.ui')
    loadUi(ui_file, dialog)

    if current_name is not None:
        dialog.label_lineEdit.setText(current_name)
    dialog.username_lineEdit.setText(queue_default_username)
    dialog.password_lineEdit.setText(queue_default_password)

    rc = dialog.exec_()
    if rc == QDialog.Rejected:
        return

    queue_program(dialog.username_lineEdit.text(), dialog.password_lineEdit.text(), dialog.label_lineEdit.text())

def save_and_queue_program():
    rc = save_to_file()
#    if rc:
#        queue_program(queue_default_username, queue_default_password, current_name)

#def queue_program(username, password, label):
#    queue = ProgramQueue(username, password)
#    try:
#        rc = queue.login()
#    except:
#        rc = False
#    if not rc:
#        QMessageBox.critical(main_window, main_window.tr('Login failed'), main_window.tr('Could not log in to the program queue.'))
#        return
#
#    output = StringIO()
#    storage = SimpleFormat(output)
#    serialize(storage)
#    program_data = output.getvalue()
#    output.close()
#
#    try:
#        id = queue.upload_program(program_data, label)
#    except:
#        id = False
#    if not id:
#        QMessageBox.critical(main_window, main_window.tr('Upload failed'), main_window.tr('Could not upload program to queue.'))
#        return
#    try:
#        queue.logout()
#    except:
#        pass
#    QMessageBox.information(main_window, main_window.tr('Uploaded program'), main_window.tr('Uploaded program (%d) successfully.') % id)

main_window.actionQueue_Program.triggered.connect(queue_dialog)
main_window.save_and_queue_program_pushButton.clicked.connect(save_and_queue_program)


main_window.actionExit.triggered.connect(main_window.close)

# confirm closing of application
def closeEvent(event):
    if not sigint_called:
        button = QMessageBox.question(main_window, main_window.tr('Close application'), main_window.tr('Do you really want to close the application?'), QMessageBox.Cancel | QMessageBox.Close)
    else:
        button = QMessageBox.Close
    if button == QMessageBox.Close:
        event.accept()
    else:
        event.ignore()
main_window.closeEvent = closeEvent


#main_window.show()
main_window.showMaximized()

execute_default_pose()

sys.exit(app.exec_())
