import matplotlib.pyplot as plt
import numpy as np

from ..src.path_planning.ros2.bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Planner

#from ..src.LocalPlanning.bayesian_inference_planner import Bayesian_Inference_Planner
#from Planning.include.LocalPlanning.bayesian_inference_gains import Bayesian_Inference_Gains

class Bayesian_Inference_Gains:
    def __init__(self, max_angle_change_gain, std_dvt_track_width_gain, 
                       std_dvt_left_right_cones, max_wrong_color_gain, sqd_diff_path_len_sensor_range):
        self.max_angle_change_gain = max_angle_change_gain 
        self.std_dvt_track_width_gain = std_dvt_track_width_gain
        self.std_dvt_left_right_cones = std_dvt_left_right_cones
        self.max_wrong_color_gain = max_wrong_color_gain
        self.sqd_diff_path_len_sensor_range = sqd_diff_path_len_sensor_range
        pass

class Cone:
    def __init__(self, x_position_track_reference_frame, y_position_track_reference_frame, color, confidence):
        self.x_position_track_reference_frame = x_position_track_reference_frame
        self.y_position_track_reference_frame = y_position_track_reference_frame
        self.color = color
        self.confidence = confidence
class Global_Track_Map:
    def __init__(self, cones):
        self.cones = cones
class  Vehicle_Pose:
    def __init__(self,x_position_track_reference_frame, y_position_track_reference_frame, yaw_position_track_reference_frame):
        self.x_position_track_reference_frame = x_position_track_reference_frame
        self.y_position_track_reference_frame = y_position_track_reference_frame
        self.yaw_position_track_reference_frame = yaw_position_track_reference_frame

cones_raw = np.genfromtxt('Planning/tests/mapa.csv',delimiter=',',skip_header=1)
gains = Bayesian_Inference_Gains(4,0,0,24,0)
local_planner = Bayesian_Inference_Planner(gains)

local_cones_raw = []
position = np.array([-14.88217243,146.67315613])
yaw = 0
pose = Vehicle_Pose(position[0],position[1], yaw)
#fig, (ax1, ax2) = plt.subplots(2,1)

while True:
    plt.cla()
    local_cones_raw = []
    for cone_raw in cones_raw:
        if np.linalg.norm(cone_raw[:2]-position)<=15:
            cone = Cone(cone_raw[0], cone_raw[1], cone_raw[2], 0.5)
            local_cones_raw.append(cone)
            if cone_raw[2] == 0:
                plt.scatter(cone_raw[0], cone_raw[1], color='b')
            else:
                plt.scatter(cone_raw[0], cone_raw[1], color='y')

    local_track = Global_Track_Map(local_cones_raw)
    #print(np.cos(yaw),np.sin(yaw))
    path = local_planner.get_interpolated_path(cones_raw, pose)
    plt.quiver(position[0],position[1], np.cos(yaw), np.sin(yaw), color="k")
    plt.plot(path[:,0],path[:,1], color="r")
    plt.scatter(position[0],position[1], color="k")
    #ax2.scatter(position[0],position[1], color="k")
    #ax2.quiver(position[0],position[1], np.cos(yaw), np.sin(yaw), color="k")
    #ax2.plot(path[:,0],path[:,1], color="r")
    #ax2.scatter(cones_raw[:,0], cones_raw[:,1], color="b")
    plt.draw()
    plt.pause(0.3)

    position = path[1]
    yaw = np.arccos((path[1]-path[0])[0]/np.linalg.norm(path[1]-path[0]))
    pose = Vehicle_Pose(position[0],position[1], yaw)