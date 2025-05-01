from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    # xacro path
    xacro_path = PathJoinSubstitution([
        FindPackageShare('sim2fruit_description'),
        'urdf/robots/mycobot_280.urdf.xacro'
    ])

    # robot from xacro
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' use_gazebo:=true use_camera:=false use_gripper:=false'
    ])
    robot_description = {'robot_description': robot_description_content}

    # gazebo launch command
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # state publisher for the robot
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mycobot_280',
        ],
        output='screen'
    )
    
    # define model for plant
    plant_model_path = PathJoinSubstitution([
    FindPackageShare('sim2fruit_description'),
    'models/plant1/plant1.sdf'
     ])

#    spawn_plant_cmd = Node(
 #   	package='gazebo_ros',
  #  	executable='spawn_entity.py',
   # 	arguments=[
    #    '-file', plant_model_path,
    #    '-entity', 'plant1',
    #    '-x', '0.5', '-y', '0.5', '-z', '0.5'
    #],
    #output='screen'
#)
    spawn_sun_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', '/usr/share/gazebo-11/models/sun/model.sdf',
            '-entity', 'test_box',
            '-x', '0.5', '-y', '0.0', '-z', '1.0'
        ],
        output='screen'
    )
    

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        rsp_node,
        spawn_entity,
        spawn_sun_cmd
    ])

