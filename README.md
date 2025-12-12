# ik_solver_moveit_adapter

A simple ROS2 adapter node to interface MoveIt2 srv_kinematics_plugin with the ik_solver package.

It allows using custom IK plugins from the ik_solver package with MoveIt2 planners.

The default installation also downloads an analytical IK solver for Universal Robots.

## Installation

- Download this package and its dependencies:
	```
	cd {your_ws}/src
	git clone https://github.com/MerlinLaboratory/ik_solver_moveit_adapter
	vcs import < ik_solver_moveit_adapter/deps.repos
	git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/cnr_common.git
	sudo apt update
	sudo apt -y install libboost-all-dev libeigen3-dev libyaml-cpp-dev libpoco-dev liblog4cxx-dev libgtest-dev
	rosdep install --from-paths src --ignore-src -r -y
	```

- Set up ```cnr_param` for ROS2: go to src/cnr_common/cnr_param/CMakelists.txt and change this line

	```
	option(COMPILE_ROS2_MODULE "ROS 2 INTEGRATION SUPPORT" OFF)
	```
	to
	```
	option(COMPILE_ROS2_MODULE "ROS 2 INTEGRATION SUPPORT" ON)
	```

- Build the ws:
	```
	cd {your_ws}
	colcon build --symlink-install
	source install/setup.bash
	```

- Export some environmental variable to make ```cnr_param``` and ```cnr_logger``` work:
	```
	echo 'export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"' >> ~/.bashrc
	echo 'export IK_SOLVER_LOGGER_CONFIG_PATH="{path to your ws}/src/ik_solver/logger_config.yaml"' >> ~/.bashrc
	source ~/.bashrc
	```

**Note!** The installation procedure automatically download a patched version of MoveIt2 **srv_kinematic_plugin** because the original version is bugged as it gets caught in a deadlock when it tries to call an external service. See this [issue](https://github.com/issues/created?issue=moveit%7Cmoveit2%7C3633) for details. 


## Configuration of your MoveIt package

To use the custom IK in MoveIt you need to load the ```srv_kinematics_plugin``` instead of the default IK solver in your moveit_config package.

To do so, open ```{your moveit config package}/config/kinematics.yaml` and set 
	```
	kinematics_solver: srv_kinematics_plugin/SrvKinematicsPlugin
	kinematics_solver_service_name: "/ik_adapter_service"
	```

## Usage

To launch an example with a UR5e robot:

- First spawn your UR5e robot to make the ```/robot_description``` topic available

- Then ```ros2 launch ik_solver_moveit_adapter ur5e_analytical_ik.launch.py```

- Try to request an IK from terminal:
	```
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "ik_request:
  group_name: 'manipulator'
  robot_state:
    joint_state:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
      position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
      velocity: []
      effort: []
    multi_dof_joint_state:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      joint_names: []
      transforms: []
      twist: []
      wrench: []
    attached_collision_objects: []
    is_diff: false
  constraints:
    name: ''
    joint_constraints: []
    position_constraints: []
    orientation_constraints: []
    visibility_constraints: []
  avoid_collisions: false
  ik_link_name: ''
  pose_stamped:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: 'base_link'
    pose:
        position:
          x: 0.492
          y: 0.133
          z: 0.338                           
        orientation:
          x: -0.707
          y: 0.707
          z: 0.001
          w: 0.0
  ik_link_names: [tip]
  pose_stamped_vector: []
  timeout:
    sec: 0
    nanosec: 0"
	```

## Configuration

To configure your IK plugin see [here](config/example_ur5e.yaml), [here](launch/ur5e_analytical_ik.launch.py), and [here](https://github.com/JRL-CARI-CNR-UNIBS/ur_ik_solver/tree/ros2/config).