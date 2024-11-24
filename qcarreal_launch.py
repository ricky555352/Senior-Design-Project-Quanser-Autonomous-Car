#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([

		# QCar Vehicle Control Node
		Node(
			package='qcar',  # Name of the package
			node_executable='vehiclecontrol',  # Name of the vehicle control node
			name='vehicle_control',
			output='screen',
			parameters=[
				{'calibrate': True}  # Add parameters if any
			]
		),
  
		# QCar Camera Node
		Node(
			package='qcar',  # Name of the package
			node_executable='camera',  # Name of the camera node
			name='camera',
			output='screen',
			parameters=[
				{},  # Add parameters if any
			]
		),

		# QCar Stop Sign Node
		Node(
			package='qcar',  # Name of the package
			node_executable='stopsign',  # Name of the detecting lane node
			name='stop_sign',
			output='screen',
			parameters=[
				{'param_name_2': 'param_value_2'},  # Add parameters if any
			]
		),
  
		# QCar Traffic Law Detection Node
		Node(
			package='qcar',
			node_executable='trafficlight',  # Name of the package
			name='traffic_light',
			output='screen',
			parameters=[
				{},  # Add parameters if any
			]
		),



		])
	
	'''



  
		# QCar Detecting Lanes Node
		Node(
			package='qcar',  # Name of the package
			node_executable='detectinglanes',  # Name of the detecting lane node
			name='detecting_lanes',
			output='screen',
			parameters=[
				{'param_name_2': 'param_value_2'},  # Add parameters if any
			]
		),

		Node(
			package='qcar',  # Name of the package
			node_executable='main_control',  # Name of the camera node
			name='main_control',
			output='screen',
			parameters=[
				{},  # Add parameters if any
			]
		),
				
		Node(
			package='qcar',  # Name of the package
			node_executable='speed_controller',  # Name of the camera node
			name='speed_controller',
			output='screen',
			parameters=[
				{},  # Add parameters if any
			]
		),

		Node(
			package='qcar',  # Name of the package
			node_executable='steering_controller',  # Name of the camera node
			name='steering_controller',
			output='screen',
			parameters=[
				{},  # Add parameters if any
			]
		),
		
		# QCar Detecting Lanes Node
		Node(
			package='qcar',  # Name of the package
			node_executable='detectinglanes',  # Name of the detecting lane node
			name='detecting_lanes',
			output='screen',
			parameters=[
				{'param_name_2': 'param_value_2'},  # Add parameters if any
			]
		),
		

		
		# QCar Traffic Law Detection Node
		Node(
			package='qcar',
			node_executable='stopsign',  # Name of the package
			name='stop_sign',
			output='screen',
			parameters=[
				{'param_name_1': 'param_value_1'},  # Add parameters if any
			]
		),

	'''