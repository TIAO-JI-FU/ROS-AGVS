# ROS-AGVS
## Schedule Software
No.| Features              | State    | Date   |
:-:|:---------------------:|:--------:|:------:|
1  | Flow chart            | Finish   |7/22    |
2  | AGVS model            | Finish   |7/23    |
3  | Goods model           | Finish   |7/23    |
4  | Map model             | Finish   |7/24    |
5  | AGVS controller       | Finish   |7/27    |
6  | Read Lidar data       | Finish   |7/27    |
7  | Move AGVS             | Finish   |7/28    |
8  | SLAM map              | Finish   |7/31    |
9  | Navigation            | Finish   |8/04    |
10 | Location calculation  | N/A      | N/A    |
11 | Angle calculation     | N/A      | N/A    |
12 | Distance calculation  | N/A      | N/A    |
13 | Imformation collection| N/A      | N/A    |
14 | Instruction decode    | N/A      | N/A    |

## Schedule Hardware
No.| Features              | State    | Date   |
:-:|:---------------------:|:--------:|:------:|
1  | Robot move (Raspberry)| N/A      | N/A    |
2  | Lidar                 | N/A      | N/A    |
3  | Image recognition     | N/A      | N/A    |
4  | Movement correction   | N/A      | N/A    |
5  | Arduino control       | N/A      | N/A    |
6  | Integration           | N/A      | N/A    |
7  | Debug                 | N/A      | N/A    |

## Description
### agv_description
This folder is model of agv.
### agvs_slam
This folder is slam, use gmapping.
### agvs_navigation
This folder is navigation, we have to send goal.
### agvs_teleop
This folder is using keybord to control agv.
### agvs_control
This folder is controller of agv.
### cmd_vel_transfer_to_agvs
This folder is change cmd_vel to our agvs velocity.
### ira_laser_tools
This folder is mix two laser scan.  

## How to use
### slam
  1.Lanuch model
  
    roslaunch agv_description display_agv_base_xacro.launch 
  2.Lanuch mix two scan
  
    roslaunch ira_laser_tools laserscan_multi_merger.launch 
  3.Run tf
  
    rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0.0 0.0 map base_link 100 
  4.Lanuch slam
  
    roslaunch agvs_slam agvs_slam.launch
  5.Lanuch teleop

    roslaunch agvs_teleop agvs_teleop.launch
### Navigation
  1.Lanuch model
  
    roslaunch agv_description display_agv_base_xacro.launch 
  2.Lanuch mix two scan
  
    roslaunch ira_laser_tools laserscan_multi_merger.launch 
  3.Lanuch navigation
  
    roslaunch agvs_navigation agvs_navigation.launch 
  4.Send goal

    rostopic pub /move_base/goal

