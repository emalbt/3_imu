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
#include <ros/package.h>
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


/********************
*	    NLOPT		*
********************/
#include <nlopt.hpp>

typedef struct {
    double q0, q1, q2;	
} angles_data;


typedef struct {
    double Q0, Q1, Q2, Q3;	
} quaternion_data;


// double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
// {
//     angles_data *d = reinterpret_cast<angles_data*>(my_func_data);
// 	// angles old 
// 	double 	q0 = d->q0; 
// 	double 	q1 = d->q1; 
// 	double 	q2 = d->q2;

// 	// float sum  = std::pow(x[0]-q0, 2) + std::pow(x[1]-q1, 2) + std::pow(x[2]-q2, 2); 
// 	float sum  = std::pow(x[0], 2) + std::pow(x[1], 2) + std::pow(x[2], 2); 
//     return std::sqrt(sum);
// }

// double myvconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
// {
//     quaternion_data *d = reinterpret_cast<quaternion_data*>(data);
// 	float Q0 = d->Q0;
// 	float Q1 = d->Q1;
// 	float Q2 = d->Q2;
// 	float Q3 = d->Q3;


// 	Eigen::Matrix3d R_Q;
// 	R_Q << 	1- 2*(Q2*Q2 + Q3*Q3),  2*(Q1*Q2 - Q0*Q3),      	2*(Q1*Q3 + Q0*Q2),
// 			2*(Q1*Q2 + Q0*Q3),     1 - 2*(Q1*Q1 + Q3*Q3),	2*(Q2*Q3 + Q0*Q1),
// 			2*(Q1*Q3 + Q0*Q2),     2*(Q2*Q3 + Q0*Q1), 		1 - 2*(Q1*Q1 + Q2*Q2); 



//    	float c0 = cos(x[0]);
//    	float c1 = cos(x[1]);
//    	float c2 = cos(x[2]);
//    	float s0 = sin(x[0]);
//    	float s1 = sin(x[1]);
//    	float s2 = sin(x[2]);

//    	Eigen::Matrix3d R_q;
//    	R_q << 	c0*c1*c2 - s0*s2,  -c0*c1*s2 - s0*c2, c0*s1,
//    			s0*c1*c2 + c0*s2,  -s0*c1*s2 + c0*c2, s0*s1,
//    			     -s1*c2,	           s1*s2,	  c1;

//    	Eigen::Matrix3d R,A;
//    	R = R_q - R_Q;

//    	A = R * R.transpose(); 

//    	return( std::sqrt( A(0,0)+ A(1,1)+A(2,2) ) );
// }


double constraint_x0_up(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (x[0]-2*M_PI);
}
double constraint_x0_down(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (-x[0]-2*M_PI);
}


double constraint_x1_up(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (x[1]-2*M_PI);
}
double constraint_x1_down(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (-x[1]-2*M_PI);
}


double constraint_x2_up(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (x[2]-2*M_PI);
}
double constraint_x2_down(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    return (-x[2]-2*M_PI);
}


double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    quaternion_data *d = reinterpret_cast<quaternion_data*>(data);
	float Q0 = d->Q0;
	float Q1 = d->Q1;
	float Q2 = d->Q2;
	float Q3 = d->Q3;


	Eigen::Matrix3d R_Q;
	R_Q << 	1- 2*(Q2*Q2 + Q3*Q3),  2*(Q1*Q2 - Q0*Q3),      	2*(Q1*Q3 + Q0*Q2),
			2*(Q1*Q2 + Q0*Q3),     1 - 2*(Q1*Q1 + Q3*Q3),	2*(Q2*Q3 - Q0*Q1),
			2*(Q1*Q3 - Q0*Q2),     2*(Q2*Q3 + Q0*Q1), 		1 - 2*(Q1*Q1 + Q2*Q2); 

   	float c0 = cos(x[0]); //x[0] = Z
   	float c1 = cos(x[1]); //x[1] = Y
   	float c2 = cos(x[2]); //x[2] = X
   	float s0 = sin(x[0]);
   	float s1 = sin(x[1]);
   	float s2 = sin(x[2]);

   	Eigen::Matrix3d R_q;
	  	// R_q << 	c0*c1*c2 - s0*s2,  -c0*c1*s2 - s0*c2, c0*s1,
  		// 	s0*c1*c2 + c0*s2,  -s0*c1*s2 + c0*c2, s0*s1,
  		// 	     -s1*c2,	           s1*s2,	  c1;
   	float th = 0.05;
   	if(std::abs(Q0) > std::abs(Q2) + th )
   	{	
	   	// pitch
	   	R_q << 	c0*c1,  -s0*c2 + c0*s1*s2,  s2*s0 + c0*s1*c2,
	   			s0*c1,   c0*c2 + s0*s1*s2, -c0*s2 + s0*s1*c2,
	   			  -s1,	            c1*s2,             c1*c2;
   	}
   	else
   	{
	   	R_q << 	-c0*c1,  -s0*c2 + c0*s1*s2,  s2*s0 + c0*s1*c2,
	   			-s0*c1,   c0*c2 + s0*s1*s2, -c0*s2 + s0*s1*c2,
	   			  -s1,	            -c1*s2,             -c1*c2; 
   	}

   	Eigen::Matrix3d R,A;
   	R = R_q - R_Q;
   	// std::cout << "R:\n" << R <<std::endl;
   	A = R * R.transpose(); 
   	return( std::sqrt( A(0,0)+ A(1,1)+A(2,2) ) );
}



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
	// Parse parameters
	std::string path_compass= ros::package::getPath("imu3_description") + "/file_txt/";


    // Communication Parameters
	body.p_.port 	 = (char*)"/dev/ttyUSB0";
	body.p_.baudRate = 2000000;
	body.p_.nIMU	 = 17;
	body.p_.byteIMU	 = 20; 

	// init communication
	body.initCompass(path_compass);
	body.initPSoC();

	// Madgwick filter Parameters
	body.z_.thGyro 	 	= 15.0; //20.0;
	body.z_.beta		= 2;
	body.z_.sampleFreq  = 70; //70.0;


	// IMU chains for the IMUGL filter
	std::vector<int> a,b,c,d,e;
	// right arm
	a.push_back(0);
	a.push_back(1);
	a.push_back(2);

	body.addChain(a);
	body.checkChains();


	// body.initialOffset();


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


	nlopt::opt opt(nlopt::LN_COBYLA, 3);

	// std::vector<double> lb(2);
	// lb[0] = -HUGE_VAL; lb[1] = 0;
	// opt.set_lower_bounds(lb);

	angles_data angles_old;
	angles_old.q0 = 0;
	angles_old.q1 = 0;
	angles_old.q2 = 0;

	quaternion_data Q;
	Q.Q0 = 1;
	Q.Q1 = 0;
	Q.Q2 = 0;
	Q.Q3 = 0;

	// opt.set_min_objective(myvfunc, &angles_old);
	opt.set_min_objective(myvfunc, &Q);

	// opt.add_inequality_constraint(myvconstraint, &q, 1e-1);
	// opt.add_inequality_constraint(myvconstraint, &angles_old, 1e-12);
	
	opt.add_inequality_constraint(constraint_x0_up, NULL ,  1e-12);
	opt.add_inequality_constraint(constraint_x0_down,NULL , 1e-12);
	opt.add_inequality_constraint(constraint_x1_up,NULL , 1e-12);
	opt.add_inequality_constraint(constraint_x1_down,NULL , 1e-12);
	opt.add_inequality_constraint(constraint_x2_up,NULL , 1e-12);
	opt.add_inequality_constraint(constraint_x2_down,NULL , 1e-12);

	opt.set_xtol_rel(1e-18);
	// opt.set_xtol_rel(10);
	

	std::vector<double> x(3);
	x[0] = 0; 
	x[1] = 0;
	x[2] = 0;
	
	double minf;
	nlopt::result result = opt.optimize(x, minf);

	std::cout << "result " << result << std::endl;
	std::cout << "x " << x[0]  << "  " << x[1] <<  "  " << x[2] << std::endl;
	std::cout << "minf " << minf << std::endl;
	getchar();


	while(1)
	{	

		absTime = boost::posix_time::microsec_clock::local_time();
       		
		body.computeAngles(angles);
		
		Q.Q0 = body.Q_(0,0);
		Q.Q1 = body.Q_(0,1);
		Q.Q2 = body.Q_(0,2);
		Q.Q3 = body.Q_(0,3);
		
		result = opt.optimize(x, minf);
		
		std::cout << "result " << result << std::endl;
		std::cout << "minf " << minf << std::endl;
		// body.printIMUangles();
  		
		angles_old.q0 = x[0];
		angles_old.q1 = x[1];
		angles_old.q2 = x[2];
		
		for(int i=0; i<3; i++)
		{
			if(x[i]<-M_PI)
				x[i] += 2*M_PI;

			if(x[i]>M_PI)
				x[i] -= 2*M_PI;  
		}

		std::cout << "x " << x[0]*180/M_PI  << "  " << x[1]*180/M_PI <<  "  " << x[2]*180/M_PI << std::endl;
		// getchar();


		
		// chest
		my_joint_state.position[0]  =  0;
		my_joint_state.position[1]  =  0;  
		my_joint_state.position[2]  =  0; 

		// head
		my_joint_state.position[3]  =  0; //angles(1,0); 
		my_joint_state.position[4]  =  0; //angles(1,1); 
		my_joint_state.position[5]  =  0; //angles(1,2);

		// leg right thigh
		my_joint_state.position[6]  = x[1];//-angles(0,2);
		my_joint_state.position[7]  = x[2];//-angles(0,1);
		my_joint_state.position[8]  = x[0]; //angles(0,0);



		my_joint_state.header.stamp = ros::Time::now();
		pub_joints.publish( my_joint_state );
		rate.sleep();

		firstDataTime = boost::posix_time::microsec_clock::local_time();
		relTime = absTime - firstDataTime; 
		// std::cout << "TIME: " << relTime << std::endl;
	
	}	


	return 0;
}






