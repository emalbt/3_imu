/*
	IMUGL - IMU GLOVE LIBRARY
	
    Copyright (C) 2016	Emanuele Luberto (emanuele.luberto@gmail.com)
	
	Author affiliation:
		Emanuele Luberto - Research Center “E.Piaggio”,School of Engineering,University of Pisa 
							from 01-april-2016 to current


    IMUGL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or 
	any later version.

    IMUGL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FORz A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SHOG. If not, see <http://www.gnu.org/licenses/>.
*/

#include "IMUGL.h"



// ============================================================================ Constructor#2
IMUGL::IMUGL()
// IMUGL::IMUGL(psocVariables p_constructor, bool flag)
{
	/********************
	*					*
	*	CODE VERSION	*
	*					*
	********************/
	/*
		Release date: 20-apr-2016
		Version: vMajorVersion.MinorVersion.Patcsh
		Release Version: v0.3.20
	*/
	time_t rawtime;
	struct tm * timeinfo;
	int majorVersion;
	int minorVersion;
	int patchVersion;
	std::stringstream ssVersion;
	
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	//tm_year (int)	--> years since 1900
	//tm_mon (int)	--> months since January (0-11)
	//tm_mday (int)	--> day of the month (1-31)
	majorVersion = (1900 + timeinfo->tm_year) - 2016;
	minorVersion = timeinfo->tm_mon;
	patchVersion = timeinfo->tm_mday;
	
	ssVersion.str("");
	ssVersion<<"v"<<majorVersion<<"."<<minorVersion<<"."<<patchVersion<<"\n";
	
	std::cout<<"\nWelcome\n\tIMUGL "<<ssVersion.str()<<"\n";
}


// =============================================================================================
//																					 Desctructor
// =============================================================================================
IMUGL::~IMUGL()
{
	//nothing to be done
	std::cout << "\n\n\nSHUTDOWN\n\n\n";
}




/**********************************************************************************************/
/*																					  		  */
/*																					  		  */
/*					                  		 FUNCTIONS                                        */
/*																					    	  */
/*																					  		  */
/**********************************************************************************************/


// =============================================================================================
//																						InitPSoC
// =============================================================================================
void IMUGL::initPSoC()
{

	// init public variables
	Acc_.resize(p_.nIMU, 3);
	Acc_Old_.resize(p_.nIMU,3);
	Gyro_.resize(p_.nIMU, 3);
	Gyro_Old_.resize(p_.nIMU,3);
	Gyro_Bias_.resize(p_.nIMU,3);
	Gyro_Bias_.setZero();
	IMUs_Angles_.resize(p_.nIMU,3);
	Q_.resize(p_.nIMU,4);
	count_IMU_Failure_.resize(p_.nIMU);

	z_.betaVector.resize(p_.nIMU);
	count_ = 0;

	// Initialize glove quaternion matrix   
	Q_.resize(p_.nIMU, 4);
	Q_joint_.resize(p_.nIMU,4);
	Q_off_.resize(p_.nIMU,4);
	for( int i = 0; i < p_.nIMU; ++i)
	{
	    for( int j = 0; j < 4; ++j)
	    {
	    	Q_(i,j) = 0;
	    	Q_off_(i,j) = 0;
	    	Q_joint_(i,j) = 0;
	    	if( j==0 )
	    	{
	    		Q_(i,j) = 1;
	        	Q_off_(i,j) = 1;
	        	Q_joint_(i,j) = 1;
	    	}
	    }

	    z_.betaVector(i) = 2; 
	}


	// Init Communication
	usleep(20000);
	sizeBuffer_ = p_.nIMU*p_.byteIMU;
	dataBuffer_ = (new uint8_t[sizeBuffer_]);

	serialPort_ = (new boost::asio::serial_port(ioService_));
	serialPort_->close();
	serialPort_->open(p_.port);
	serialPort_->set_option(boost::asio::serial_port_base::baud_rate(p_.baudRate));
	serialPort_->set_option(boost::asio::serial_port_base::character_size(8));
	serialPort_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	serialPort_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	serialPort_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	tcflush(serialPort_->lowest_layer().native_handle(), TCIOFLUSH);

	// int sizestartBuffer = 1;
	// uint8_t* startBuffer;
	// bool f = true;

	std::cout<< "\r\nCOMMUNICATION ESTABLISHED WITH THE PSOC5 \n";
	// wait PSCO		
	// while(f)
	// {
		// boost::asio::write(*serialPort_,boost::asio::buffer(new char('r'),1));
		// sleep(3);
	// 	boost::asio::read(*serialPort_, boost::asio::buffer(startBuffer, sizestartBuffer));
	// 	std::cout <<"startBuffer " << (int) startBuffer[0] << "\r\n" 
	// 	if (startBuffer == 255)
	// 		f = false;
	// }

	 // char*   port; 
  //   // baudRate 
  //   int     baudRate;
  //   // # of IMU connected
  //   int     nIMU;
  //   // buffer size for one IMU
  //   int     byteIMU;   

	std::cout << "p_.port: " << p_.port <<std::endl;
	std::cout << "p_.baudRate: " << p_.baudRate<<std::endl;
	std::cout << "p_.nIMU: " << p_.nIMU <<std::endl;
	std::cout << "p_.byteIMU: " << p_.byteIMU <<std::endl;



	boost::asio::write(*serialPort_,boost::asio::buffer(new char('?'),1));
	boost::asio::streambuf dataStreamBuf;
	boost::asio::read_until(*serialPort_,dataStreamBuf,'\n');		
	std::string strData(boost::asio::buffer_cast<const char*>(dataStreamBuf.data()));
	std::vector<std::string> strSensorData;//sensor data in string format
	boost::split(strSensorData, strData, boost::is_any_of("\n"));
	std::cout << "Firmware Version: " << strData << std::endl;
    getchar();

	readPSoC();
	Acc_Old_ = Acc_;
	Gyro_Bias_ = Gyro_;
}


// =============================================================================================
//																						InitPSoC
// =============================================================================================
void IMUGL::stopPSoC()
{
	std::cout<< "\r\n\nSTOP COMMUNICATION\n";
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('r'),1));
	serialPort_->close();
}


// =============================================================================================
//																						ReadPsoC
// =============================================================================================
void IMUGL::readPSoC()
{
	// IMU glove
	// read serial port buffer
	boost::asio::write(*serialPort_,boost::asio::buffer(new char('<'),1));
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('\r'),1));
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('\n'),1));

	// IMUKUKA
	// boost::asio::write(*serialPort_,boost::asio::buffer(new char('?'),1));

	boost::asio::read(*serialPort_, boost::asio::buffer(dataBuffer_, sizeBuffer_));

	if (_DEBUG_DATA_BUFFER_)
	{
		for(int i=0; i<sizeBuffer_; i++ )
		{
		    std::cout <<i<<" " << (double) dataBuffer_[i] << "\r\n";  
		}		 	
	}

	// record data in the Matrix
	int16_t		acc [p_.nIMU][3];
	int16_t		gyro[p_.nIMU][3];

	int 		rateAcc = 1;
	float 		scaleAccFactor  = 0.000061037 * rateAcc;  // *2  = form 2g to 4g  
	int 		rateGyro = 8;
	float 		scaleGyroFactor = 0.007629627 * rateGyro; // " *8 " = from 250°/s to 2000°/s;

	// scaleAccFactor = 1;
	// scaleGyroFactor= 1;

	for(int i=0; i<p_.nIMU; i++)
	{
		acc[i][0] = (dataBuffer_[1+(p_.byteIMU*i)]<<8 | dataBuffer_[2+(p_.byteIMU*i)]);
        acc[i][1] = (dataBuffer_[3+(p_.byteIMU*i)]<<8 | dataBuffer_[4+(p_.byteIMU*i)]);
        acc[i][2] = (dataBuffer_[5+(p_.byteIMU*i)]<<8 | dataBuffer_[6+(p_.byteIMU*i)]);


        gyro[i][0] = (dataBuffer_[7 +(p_.byteIMU*i)]<<8 | dataBuffer_[8 +(p_.byteIMU*i)]);
        gyro[i][1] = (dataBuffer_[9 +(p_.byteIMU*i)]<<8 | dataBuffer_[10+(p_.byteIMU*i)]);
        gyro[i][2] = (dataBuffer_[11+(p_.byteIMU*i)]<<8 | dataBuffer_[12+(p_.byteIMU*i)]);
        
	
        Acc_(i,0) = (double)  acc[i][0] * scaleAccFactor;
        Acc_(i,1) = (double)  acc[i][1] * scaleAccFactor;
        Acc_(i,2) = (double)  acc[i][2] * scaleAccFactor;

        Gyro_(i,0) = (double) gyro[i][0] * scaleGyroFactor;
        Gyro_(i,1) = (double) gyro[i][1] * scaleGyroFactor;
        Gyro_(i,2) = (double) gyro[i][2] * scaleGyroFactor;

        Gyro_(i,0) -= Gyro_Bias_(i,0);
        Gyro_(i,1) -= Gyro_Bias_(i,1);
        Gyro_(i,2) -= Gyro_Bias_(i,2);

        if(_DEBUG_REAL_DATA_)
        {
        	std::cout << i <<"  acc:\t\t" << Acc_(i,0) << "\t" << Acc_(i,1) << "\t" << Acc_(i,2)<<"\r\n"; 
        	std::cout << " " <<"  gyro:\t" << Gyro_(i,0) << "\t" << Gyro_(i,1) << "\t" << Gyro_(i,2)<<"\r\n";
        }
	}

	if(_DEBUG_REAL_DATA_)
	{
		// std::cout <<"\n\n";
	}
}





// =============================================================================================
//																				 MadgwickGeneral
// =============================================================================================
void IMUGL::MadgwickGeneral(int P, int N)
{
	//come N vede P;  N è il mio d (wordl), P è il mio s (sensor)
	// ad esempio N=imu1, P=imu0, quindi come la IMU 1 vede la IMU 0 

	// float recipNorm;
	// float qDot1, qDot2, qDot3, qDot4;
	float q1, q2 ,q3 ,q4;
    
    float dx, dy, dz;
    float sx, sy, sz;


    Eigen::Vector3d aP, aN, gPpartial, gNpartial;
    Eigen::Vector4d gP, gN, g, qL;

    Eigen::Vector3d fa;
    Eigen::MatrixXd Ja(3,4); 
    Eigen::Vector4d  qdot;

   	Eigen::Vector4d Napla;

	aP(0)  = Acc_(P,0);  
	aP(1)  = Acc_(P,1);  
	aP(2)  = Acc_(P,2);

	aN(0)  = Acc_(N,0);  
	aN(1)  = Acc_(N,1);  
	aN(2)  = Acc_(N,2);

	// aN(0)  = 0;  
	// aN(1)  = 0;  
	// aN(2)  = 1;
 	
 	aP = aP / aP.norm();
	aN = aN / aN.norm();

	gP(0)  = 0; 
	gP(1)  = Gyro_(P,0);  
	gP(2)  = Gyro_(P,1);  
	gP(3)  = Gyro_(P,2);

    gN(0)  = 0; 
    gN(1)  = Gyro_(N,0);  
    gN(2)  = Gyro_(N,1);  
    gN(3)  = Gyro_(N,2);

    // gN(0)  = 0; 
    // gN(1)  = 0;  
    // gN(2)  = 0;  
    // gN(3)  = 0;

	gP = gP*(M_PI/180);
	gN = gN*(M_PI/180);

	//qL local quaternion
	qL << Q_(P,0), Q_(P,1), Q_(P,2), Q_(P,3); 

	q1 = qL(0);  
	q2 = qL(1); 
	q3 = qL(2); 
	q4 = qL(3);

	// rotate the angular velocity
	g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;	

	//accelerometer
	dx = aN(0); 
	dy = aN(1); 
	dz = aN(2); 
	
	sx = aP(0); 
	sy = aP(1); 
	sz = aP(2); 	

	fa(0) =  2*dx*(0.5 -q3*q3 -q4*q4) + 2*dy*(q1*q4 + q2*q3) + 2*dz*(q2*q4-q1*q3) - sx; 
	fa(1) =  2*dx*(q2*q3 -q1*q4) + 2*dy*(0.5 - q2*q2 - q4*q4) + 2*dz*(q1*q2 + q3*q4) - sy;
	fa(2) =  2*dx*(q1*q3 -q2*q4) + 2*dy*(q3*q4 - q1*q2) + 2*dz*(0.5 - q2*q2 -q3*q3) - sz; 

	// Compute the Jacobian
	Ja << 2*dy*q4-2*dz*q3,    2*dy*q3+2*dz*q4 ,        -4*dx*q3+2*dy*q2-2*dz*q1,  -4*dx*q4+2*dy*q1+2*dz*q2,
		  -2*dx*q4+2*dz*q2,   2*dx*q3-4*dy*q2+2*dz*q1, 2*dx*q2+2*dz*q4,           -2*dx*q1-4*dy*q4+2*dz*q3,
		  2*dx*q3-2*dy*q2,    2*dx*q4-2*dy*q1-4*dz*q2, 2*dx*q1+2*dy*q4-4*dz*q3,   2*dx*q2+2*dy*q3;

	// Compute the Napla
  	Napla = Ja.transpose() * fa;
				    
	qdot = 0.5*QxQ( qL,g ) - ( z_.betaVector(P)*Napla );					// XXX VERIFICARE QUESTO MEGLIO 

	qL = qL + qdot / z_.sampleFreq; 
	qL = qL /qL.norm();

	Q_(P,0) = qL(0); 
	Q_(P,1) = qL(1); 
	Q_(P,2) = qL(2);  
	Q_(P,3) = qL(3); 

} 




// =============================================================================================
//																				   InitialOffest
// =============================================================================================
void IMUGL::initialOffset()
{

	// j = 2 because of the filter needs two different oreintation
	for(int j=0; j<2; j++)
	{
		for (int k=0; k < _OFFSET_STEP_; k++) 
		{
			readPSoC();
			
			// Filter the gyroscope
    		for (int i=0; i<p_.nIMU; i++)
    		{
				if (std::abs(Gyro_(i,0))< z_.thGyro)  Gyro_(i,0) = 0;		  
				if (std::abs(Gyro_(i,1))< z_.thGyro)  Gyro_(i,1) = 0;
				if (std::abs(Gyro_(i,2))< z_.thGyro)  Gyro_(i,2) = 0;
			}

			checkIMU();	

			// Compute the MadgwickFilter for the each IMUchains
			for (int ii = 0; ii < (int) IMUchains_.size(); ii++ )
			{
				for (int jj=0; jj < (int) IMUchains_[ii].size() - 1; jj++)
				{	
					MadgwickGeneral(IMUchains_[ii][jj],IMUchains_[ii][jj+1]); 					
				}
			}

			// Record old Gyro value
			Gyro_Old_ = Gyro_;
			std::cout<<"step init: "<< k+1 << "\r\n";
			Q_joint_= Q_;
		
			usleep(500);		
		}	
		
		// Compute angles from quaternions
		Quat2AngleTot();

		// Print IMUs_Angles offset
		printIMUangles();

		if (j<2) 
		{	
			printf("Change Hand Orientation and push ENTER \r\n");
			getchar();
		}

		// Update quaternions offset
		Q_off_ = Q_;
	}
}




// ============================================================================================= 
//																				   computeAngles
// ============================================================================================= 
void IMUGL::computeAngles(Eigen::MatrixXd& out)
{	


	out.resize(p_.nIMU, 3);
	// Read Data From PSoC
    readPSoC();

    // Filter the gyroscope
	for (int i=0; i<p_.nIMU; i++)
	{
		if (std::abs(Gyro_(i,0))< z_.thGyro)  Gyro_(i,0) = 0;		  
		if (std::abs(Gyro_(i,1))< z_.thGyro)  Gyro_(i,1) = 0;
		if (std::abs(Gyro_(i,2))< z_.thGyro)  Gyro_(i,2) = 0;
	}

    checkIMU();
    
    // Compute the MadgwickFilter for the each IMUchains
	for (int ii = 0; ii < (int) IMUchains_.size(); ii++ )
	{
		for (int jj=0; jj < (int) IMUchains_[ii].size() - 1; jj++)
		{
			MadgwickGeneral(IMUchains_[ii][jj],IMUchains_[ii][jj+1]); 
			z_.betaVector(ii) = z_.beta;
		}
	}

	// record Acc data every 100 samples
    Acc_Old_ = Acc_;
    Gyro_Old_ = Gyro_;


    offsetCorrector('s');
    Quat2AngleTot();

    out = IMUs_Angles_;
}




// ============================================================================================= 
//																				  printIMUangles
// ============================================================================================= 
void IMUGL::printIMUangles()
{	
	int k = 0;
	std::cout<<"\n";
	for (int i = 0; i < (int) IMUchains_.size(); i++ )
	{
		for (int j=0; j < (int) IMUchains_[i].size() - 1; j++)
		{
			std::cout <<k<<  "  a: " <<IMUchains_[i][j] << " " <<IMUchains_[i][j+1] << ":\t";
			std::cout << IMUs_Angles_(k,0)*(180/M_PI) << "\t" <<IMUs_Angles_(k,1)*(180/M_PI)<< "\t"<<IMUs_Angles_(k,2)*(180/M_PI)<<std::endl; 			
			k++;   
		}
	}

	std::cout<<"\n\n";	
}



// ============================================================================================= 
//																						CheckIMU
// ============================================================================================= 
void IMUGL::checkIMU()
{	
	Eigen::MatrixXd Acc_Diff(p_.nIMU,3);
	Eigen::Matrix3d Current_Acc;
	Eigen::Vector3d IMU_n;
	
	// Acc_Old_.resize(p_.nIMU, 3);
	Acc_Faliure_.resize(p_.nIMU);
	


	Acc_Diff = (Acc_ - Acc_Old_);
	
	for(int k=0; k<p_.nIMU; k++)
	{
		IMU_n(0)  = Acc_Diff(k,0);  
		IMU_n(1)  = Acc_Diff(k,1);  
		IMU_n(2)  = Acc_Diff(k,2);

		if (IMU_n.norm() == 0) 
		{
			count_IMU_Failure_(k) += 1;
			if(count_IMU_Failure_(k) == 30)
			{
				Acc_Faliure_(k) = 0; 
				std::cout<<"\r\nbad communication IMU: "<< k  << "   cont: " << count_IMU_Failure_(k) << "\r\n";
				count_IMU_Failure_(k) = 0;
				// getchar();
			}
		} 
		else 
			Acc_Faliure_(k) = 1;
		
		Current_Acc(0) = Acc_(k,0);
    	Current_Acc(1) = Acc_(k,1);
    	Current_Acc(2) = Acc_(k,2);
    	if (Current_Acc.norm() > 1.2) 
    	{
      		Acc_(k,0) = Acc_Old_(k,0);
      		Acc_(k,1) = Acc_Old_(k,1);
      		Acc_(k,2) = Acc_Old_(k,2);
			
			z_.betaVector(k) = 0;
		}
		else
			z_.betaVector(k) = z_.beta;

      	// std::cout << k <<"  acc:\t\t" << Acc_(k,0) << "\t" << Acc_(k,1) << "\t" << Acc_(k,2)<<"\r\n"; 
        // std::cout << " " <<"  gyro:\t" << Gyro_(k,0) << "\t" << Gyro_(k,1) << "\t" << Gyro_(k,2)<<"\r\n";

	}
}



// ============================================================================================= 
//																					 checkChains
// ============================================================================================= 
void IMUGL::checkChains()
{	
	// check IMUschains_
	if (IMUchains_.size() == 0 )
	{
		std::cout << "\r\033[31m\033[1mError:  IMUchains_ vector is empty! \033[0m \r\n\n";
		exit(1);
	}
	std::cout << "\n\n";
	std::cout<< "\n\r\033[32m\033[1mIMUchains_ vector is: \033[0m \r\n" ;
	for(int i=0; i < (int) IMUchains_.size(); i++)
	{
		std::cout << "chain # " << i << std::endl;
		for(int j=0; j< (int) IMUchains_[i].size(); j++)
			std::cout << " " << IMUchains_[i][j];
		
		std::cout << "\n";
	}
	std::cout << "\n\n";

}



// ============================================================================================= 
//																					    addChain
// ============================================================================================= 
void IMUGL::addChain( std::vector<int> a )
{	
	IMUchains_.push_back(a);
	std::cout<< "Chain Added:  ";
	for(int i=0; i < (int) a.size(); i++)
	{
		std::cout << " " << a[i];
	}
	std::cout<<"\n";
}



// ============================================================================================= 
//																				Useful Functions
// =============================================================================================
Eigen::Matrix3d IMUGL::rotX(float x)
{
	Eigen::Matrix3d Rx;
	x = x *(M_PI/180); // From deg to Rad
	Rx << 1 , 0, 0,
		  0, cos(x), -sin(x),
		  0, sin(x), cos(x);
	return Rx;
}
Eigen::Matrix3d IMUGL::rotY(float y)
{
	Eigen::Matrix3d Ry;
	y = y *(M_PI/180); // From deg to Rad
	Ry << cos(y) , 0, sin(y),
		  0,       1,   0,
		  -sin(y), 0, cos(y);
	return Ry;
}

Eigen::Matrix3d IMUGL::rotZ(float z)
{
	Eigen::Matrix3d Rz;
	z = z *(M_PI/180); // From deg to Rad
	Rz << cos(z), -sin(z), 0,
		  sin(z), cos(z),  0,
		  0,        0,     1;
	return Rz;
}

Eigen::Matrix3d IMUGL::rot(float x, float y, float z)
{
	Eigen::Matrix3d Rx, Ry, Rz, Rf;
	
	Rx = rotX(x);
	Ry = rotY(y);
	Rz = rotZ(z);

	Rf = Rz*Ry*Rx;
	return Rf;
}
//Skew Matrix
Eigen::Matrix3d IMUGL::skew(Eigen::Vector3d a)
{
		Eigen::Matrix3d q;
		 q(0,0) =  0;          q(0,1) = -a(2);       q(0,2) = a(1);
		 q(1,0) =  a(2);       q(1,1) =  0;          q(1,2) = -a(0);
		 q(2,0) = -a(1);       q(2,1) =  a(0);       q(2,2) = 0;
			return q;
}
//From Quaternion to Rotation Matrix
Eigen::Matrix3d IMUGL::Quat2Rot(Eigen::Vector4d Q_in)
{
	Eigen::Matrix3d RQ;
	float q0, q1, q2, q3;
	q0 = Q_in(0);
	q1 = Q_in(1);
	q2 = Q_in(2);
	q3 = Q_in(3);
	
	RQ <<         q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2),
				  2*(q1*q2 + q0*q3), q0*q0 + q2*q2 - q1*q1 - q3*q3, 2*(q2*q3 - q0*q1),
				  2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0*q0 + q3*q3 - q1*q1 - q2*q2;
	return RQ;
}

Eigen::Vector4d IMUGL::ConjQ(Eigen::Vector4d Q_in) 
{
	Eigen::Vector4d Q_out;
	Q_out(0) =  Q_in(0);
	Q_out(1) = -Q_in(1);
	Q_out(2) = -Q_in(2);
	Q_out(3) = -Q_in(3);
	return Q_out;
}

Eigen::Vector4d IMUGL::QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2) 
{
    Eigen::Vector4d Q_out;
		Q_out(0) = Q_1(0)*Q_2(0) - (Q_1(1)*Q_2(1) + Q_1(2)*Q_2(2) + Q_1(3)*Q_2(3));
		Q_out(1) = Q_1(0)*Q_2(1) + Q_1(1)*Q_2(0) + (Q_1(2)*Q_2(3) - Q_1(3)*Q_2(2));
		Q_out(2) = Q_1(0)*Q_2(2) + Q_1(2)*Q_2(0) + (Q_1(3)*Q_2(1) - Q_1(1)*Q_2(3));
		Q_out(3) = Q_1(0)*Q_2(3) + Q_1(3)*Q_2(0) + (Q_1(1)*Q_2(2) - Q_1(2)*Q_2(1));
	return Q_out;
}

Eigen::Vector4d IMUGL::Rot2Quat(Eigen::Matrix3d R_in)
{
	Eigen::Vector4d q_out;
	q_out (0) = 0.5 * sqrt(R_in(0,0) +R_in(1,1) +R_in(2,2) +1 );
	q_out (1) = 0.5 * sgn(R_in(2,1)-R_in(1,2)) * sqrt(R_in(0,0)-R_in(1,1) - R_in(2,2) + 1);
	q_out (2) = 0.5 * sgn(R_in(0,2)-R_in(2,0)) * sqrt(-R_in(0,0)+R_in(1,1) - R_in(2,2) + 1);
	q_out (3) = 0.5 * sgn(R_in(1,0)-R_in(0,1)) * sqrt(-R_in(0,0)-R_in(1,1) + R_in(2,2) + 1);

	return q_out; 
}

Eigen::Vector3d IMUGL::Quat2Angle(Eigen::Vector4d Q_in)
{
	Eigen::Vector3d Angles_out;
 		Angles_out(0) = atan2(2*Q_in(1)*Q_in(2) - 2*Q_in(0)*Q_in(3), 2*Q_in(0)*Q_in(0) + 2*Q_in(1)*Q_in(1)-1); //YAW
	 	Angles_out(1) = -asin(2*Q_in(1)*Q_in(3) + 2*Q_in(0)*Q_in(2)); //PITCH
 	 	Angles_out(2) = atan2(2*Q_in(2)*Q_in(3) - 2*Q_in(0)*Q_in(1), 2*Q_in(0)*Q_in(0) + 2*Q_in(3)*Q_in(3)-1); // ROLL
	return Angles_out; 	 	
}

Eigen::Vector3d IMUGL::Rot2Angle(Eigen::Matrix3d R_in)
{
	Eigen::Vector3d Angles_out;
 		Angles_out(0) = atan2(R_in(1,0),R_in(0,0)); //YAW
	 	Angles_out(1) = atan2(-R_in(2,0), sqrt(R_in(2,1)*R_in(2,1) + R_in(2,2)*R_in(2,2))); //PITCH
 	 	Angles_out(2) = atan2(R_in(2,1),R_in(2,2)); // ROLL
	return Angles_out; 	 	
}

void IMUGL::Quat2AngleTot()
{
	for (int k=0; k<p_.nIMU; k++) 
	{
 		IMUs_Angles_(k,0) = atan2(2*Q_joint_(k,1)*Q_joint_(k,2) - 2*Q_joint_(k,0)*Q_joint_(k,3), 2*Q_joint_(k,0)*Q_joint_(k,0) + 2*Q_joint_(k,1)*Q_joint_(k,1)-1); //YAW
	 	// IMUs_Angles_(k,1) = -asin(2*Q_joint_(k,1)*Q_joint_(k,3) + 2*Q_joint_(k,0)*Q_joint_(k,2)); //PITCH
	 	IMUs_Angles_(k,1) = -atan(   (2*Q_joint_(k,1)*Q_joint_(k,3) + 2*Q_joint_(k,0)*Q_joint_(k,2)) / (sqrt(1-(2*Q_joint_(k,1)*Q_joint_(k,3) + 2*Q_joint_(k,2)*Q_joint_(k,0)) * (2*Q_joint_(k,1)*Q_joint_(k,3) + 2*Q_joint_(k,2)*Q_joint_(k,0) )))); //PITCH
 	 	IMUs_Angles_(k,2) = atan2(2*Q_joint_(k,2)*Q_joint_(k,3) - 2*Q_joint_(k,0)*Q_joint_(k,1), 2*Q_joint_(k,0)*Q_joint_(k,0) + 2*Q_joint_(k,3)*Q_joint_(k,3)-1); // ROLL
    }
}

void IMUGL::offsetCorrector(char choice)
{
  if (choice == 'n')
    Q_joint_ = Q_;
  if (choice == 's')
  { 
    for (int k=0; k < p_.nIMU; ++k)
    {
      Q_joint_(k,0) = Q_off_(k,0)*Q_(k,0) - (-Q_off_(k,1)*Q_(k,1) - Q_off_(k,2)*Q_(k,2) - Q_off_(k,3)*Q_(k,3));
      Q_joint_(k,1) = Q_off_(k,0)*Q_(k,1) - Q_off_(k,1)*Q_(k,0) - Q_off_(k,2)*Q_(k,3) + Q_off_(k,3)*Q_(k,2);
      Q_joint_(k,2) = Q_off_(k,0)*Q_(k,2) - Q_off_(k,2)*Q_(k,0) - Q_off_(k,3)*Q_(k,1) + Q_off_(k,1)*Q_(k,3);
      Q_joint_(k,3) = Q_off_(k,0)*Q_(k,3) - Q_off_(k,3)*Q_(k,0) - Q_off_(k,1)*Q_(k,2) + Q_off_(k,2)*Q_(k,1);
    }
  }
}


void IMUGL::AngleTot2Quat()
{
	Eigen::Vector3d rpy;
	Eigen::Vector4d q;
	Eigen::Matrix3d R;
	for (int k=0; k < p_.nIMU; ++k)
    {
    	// switch (k)
    	// {
    	// 	case 0: case 1: case 3: case 4: case 6: case 7: case 9: case 10: case 12: case 13:
    	// 	  IMUs_Angles_(k,0) = 0;
    	// 	  break;
    	// 	default: 
    	// 	  break;	
    	// }

    	rpy << IMUs_Angles_(k,0),IMUs_Angles_(k,1),IMUs_Angles_(k,2);
    	rpy = - rpy*(180/M_PI);
    	R = rot (rpy(2), rpy(1), rpy(0));
    	q = Rot2Quat(R);
    	q = q/q.norm();
    	Q_joint_(k,0) = q(0);
    	Q_joint_(k,1) = q(1);
    	Q_joint_(k,2) = q(2);
    	Q_joint_(k,3) = q(3);
    }
}