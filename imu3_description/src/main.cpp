//general utilities
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
// #include <flann/flann.h>
// #include <flann/io/hdf5.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>



// utilities
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#include <boost/date_time/posix_time/posix_time.hpp>



#include <IMUGL.h>
// #include <IMUGL_world.h>

IMUGL body;


void signalHandler( int signum )
{
    std::cout << "\n\nInterrupt signal received\n";
	// Stop Communication
    body.stopPSoC();
	ros::shutdown();
}



int main(int argc, char** argv)
{	

	/********************
	*					*
	*	     ROS		*
	*					*
	********************/
	ros::init(argc, argv, "manikin", ros::init_options::NoSigintHandler);
  	ros::NodeHandle n;

  	// register signal SIGINT (CTRL+C) and signal handler  
    signal(SIGINT, signalHandler);  

	// Parse parameters
  	std::string body_name = "manikin";
  	// n.param<std::string>("hand_name", hand_name, "soft_hand");


  	ros::Publisher pub_joints = n.advertise<sensor_msgs::JointState>("joint_states", 1000); 
  	sensor_msgs::JointState my_joint_state;
  
  	my_joint_state.name.resize(9);
  	my_joint_state.position.resize(9);

  	// Init position to 0
  	for( int j = 0; j < 9; ++j)
  	{
   	 	my_joint_state.position[j] = 0;
  	}
  	getchar();

  	// Fill joint names... 
	// chest
	my_joint_state.name[0] 	=  "imu_first_joint_roll";
	my_joint_state.name[1] 	=  "imu_first_joint_pitch";
	my_joint_state.name[2] 	=  "imu_first_joint_yaw";

	// head
	my_joint_state.name[3] 	=  "imu_second_joint_roll";
	my_joint_state.name[4] 	=  "imu_second_joint_yaw";
	my_joint_state.name[5] 	=  "imu_second_joint_pitch";
	
	// leg right thigh
	my_joint_state.name[6] 	=  "imu_third_joint_roll";
	my_joint_state.name[7] 	=  "imu_third_joint_pitch";
	my_joint_state.name[8] 	=  "imu_third_joint_yaw";

	

	// Publish first message before looping
	my_joint_state.header.stamp = ros::Time::now();
	pub_joints.publish(my_joint_state);
	ros::spinOnce();

	double spin_rate = 1000;
	ros::Rate rate(spin_rate);


	/********************
	*					*
	*	    IMUGL		*
	*					*
	********************/
    // Communication Parameters
	body.p_.port 	 = (char*)"/dev/ttyUSB0";
	body.p_.baudRate = 1000000;
	body.p_.nIMU	 = 3;
	body.p_.byteIMU	 = 14; 

	// init communication
	body.initPSoC();

	// Madgwick filter Parameters
	body.z_.thGyro 	 	= 0.0; //20.0;
	body.z_.beta		= 2;
	body.z_.sampleFreq  = 1/0.008; //70.0;


	// IMU chains for the IMUGL filter
	std::vector<int> a,b,c,d,e;
	// right arm
	a.push_back(0);
	a.push_back(1);
	a.push_back(2);

	body.addChain(a);
	body.checkChains();


	body.initialOffset();


	// //Press CTRL+C to stop data stream loop
	// /********************
	// *					*
	// *	    LOOP		*
	// *					*
	// ********************/
	Eigen::MatrixXd angles;
	angles.resize(body.p_.nIMU,3);

	boost::posix_time::time_duration relTime;
	boost::posix_time::ptime firstDataTime, absTime;

	while(1)
	{	

		absTime = boost::posix_time::microsec_clock::local_time();
       		
		body.computeAngles(angles);
		body.printIMUangles();
  		
  		
		
		// chest
		my_joint_state.position[0]  =  0;
		my_joint_state.position[1]  =  0;  
		my_joint_state.position[2]  =  0; 

		// head
		my_joint_state.position[3]  =  angles(1,0); 
		my_joint_state.position[4]  =  angles(1,1); 
		my_joint_state.position[5]  =  angles(1,2);

		// leg right thigh
		my_joint_state.position[6]  = angles(0,0);
		my_joint_state.position[7]  = angles(0,1);
		my_joint_state.position[8]  = angles(0,2);



		my_joint_state.header.stamp = ros::Time::now();
		pub_joints.publish( my_joint_state );
		rate.sleep();

		firstDataTime = boost::posix_time::microsec_clock::local_time();
		relTime = absTime - firstDataTime; 
		// std::cout << "TIME: " << relTime << std::endl;
	
	}	


	return 0;
}