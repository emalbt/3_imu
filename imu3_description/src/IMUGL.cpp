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
#include <boost/date_time/posix_time/posix_time.hpp>





// ============================================================================ Constructor#2
IMUGL::IMUGL()
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
	std::cout << "\n\n\nSONO FUORI \n\n\n";
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
	IMUs_Angles_old_.resize(p_.nIMU,3);
	Mag_.resize(p_.nIMU,3);
	COM_.resize(p_.nIMU,3);
	// MagCal_.resize(p_.nIMU,3);
	MagCal_.resize(17,3);

	z_.betaVector.resize(p_.nIMU);
	count_ = 0;

	// Initialize manikin quaternion matrix   
	Q_.resize(p_.nIMU, 4);
	Q_joint_.resize(p_.nIMU,4);
	Q_off_.resize(p_.nIMU,4);
	Q_world_.resize(p_.nIMU,4);
	for( int i = 0; i < p_.nIMU; ++i)
	{
	    for( int j = 0; j < 4; ++j)
	    {
	    	Q_(i,j) = 0;
	    	Q_off_(i,j) = 0;
	    	Q_joint_(i,j) = 0;
	    	Q_world_(i,j) = 0;
	    	if( j==0 )
	    	{
	    		Q_(i,j) = 1;
	        	Q_off_(i,j) = 1;
	        	Q_joint_(i,j) = 1;
	        	Q_world_(i,j) = 1;
	    	}
	    }

	    z_.betaVector(i) = 2; 
	}

	Quaternion(0) = 1;
	Quaternion(1) = 0;
	Quaternion(2) = 0;
	Quaternion(3) = 0;

	/********************
	*					*
	*	    MAG CAL		*
	*					*
	********************/
	// MAG CAL initialize         							
	MagCal_ << 	181, 181, 169,	 
				174, 175, 163,	 
				175, 174, 163,	 
				176, 179, 166,	 
				181, 183, 170,	 
				177, 178, 166,	 
				170, 173, 159,	 
				170, 172, 159,	 
				177, 178, 167,	 
				176, 178, 166,	
				177, 177, 165,	 
				176, 177, 164,	        
				179, 179, 168,
				174, 174, 162,	 
				182, 183, 170,	        
				171, 169, 158,
 				178, 180, 168;	
	/********************
	*					*
	*	    MAG CAL		*
	*					*
	********************/
				
	for (int i=0 ; i<p_.nIMU; i++)
	{
		for (int j=0; j<3; j++)
		{
			MagCal_(i,j) =  (( (MagCal_(i,j) -128)*0.5 ) /128) + 1;
		}
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

	std::cout << "\n\n\r\033[32m\033[1m[IMUGL] initPSoC: Communication Established\033[0m\r" << std::endl;
	

	std::cout << "p_.port: " << p_.port <<std::endl;
	std::cout << "p_.baudRate: " << p_.baudRate<<std::endl;
	std::cout << "p_.nIMU: " << p_.nIMU <<std::endl;
	std::cout << "p_.byteIMU: " << p_.byteIMU <<std::endl;

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
	std::cout << "\n\n\r\033[31m\033[1m[IMUGL] stopPSoC: STOP COMMUNICATION\033[0m\r\n\n" << std::endl;
	serialPort_->close();
}


// =============================================================================================
//																				    Init Compass
// =============================================================================================
void IMUGL::initCompass(std::string path)
{
	cf_.resize(p_.nIMU, 6);
	
	if(boost::filesystem::is_regular_file(path + "Matrix_Corrector_Factors.txt"))
	{
		FILE *MCFfile = fopen( (path + "Matrix_Corrector_Factors.txt").c_str() ,"r");

		for(int i=0; i<p_.nIMU; i++ )
			fscanf(MCFfile, "%f\t%f\t%f\t%f\t%f\t%f\n", &cf_(i,0),&cf_(i,1),&cf_(i,2),&cf_(i,3),&cf_(i,4),&cf_(i,5) );

		fclose(MCFfile);
		std::cout << "\n\n\r\033[32m\033[1m[IMUGL] initCompass: Matrix_Corrector_Factors.txt  LOADED\033[0m \r" << std::endl;
	}
	else
	{	

		for(int i=0; i<p_.nIMU; i++ )
		{
			// offset
			cf_(i,0) = 0;
			cf_(i,1) = 0;
			cf_(i,2) = 0;
			// gain
			cf_(i,3) = 1;
			cf_(i,4) = 1;
			cf_(i,5) = 1;
		}

		std::cout << "\n\n\r\033[33m\033[1m[IMUGL] initCompass: Matrix_Corrector_Factors.txt   NOT LOADED\033[0m \r" << std::endl;
	}
}




// =============================================================================================
//																				   Mag Corrector
// =============================================================================================
void IMUGL::magCorrector()
{
	Eigen::Vector3d tmp;

	for (int i=0; i<p_.nIMU; i++)
	{
		tmp(0) = cf_(i,3) * (  Mag_(i,0) - cf_(i,0) );	
		tmp(1) = cf_(i,4) * (  Mag_(i,1) - cf_(i,1) );	
		tmp(2) = cf_(i,5) * (  Mag_(i,2) - cf_(i,2) );	

		Mag_(i,0) = tmp(0);
		Mag_(i,1) = tmp(1);
		Mag_(i,2) = tmp(2);
	}
}


// =============================================================================================
//																						ReadPsoC
// =============================================================================================
void IMUGL::readPSoC()
{
	// read serial port buffer
	boost::asio::write(*serialPort_,boost::asio::buffer(new char('<'),1));
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
	int16_t		mag[p_.nIMU][3];

	int 		rateAcc = 2;
	float 		scaleAccFactor  = 0.000061037 * rateAcc;  // *2  = form 2g to 4g  
	int 		rateGyro = 8;
	float 		scaleGyroFactor = 0.007629627 * rateGyro; // " *8 " = from 250°/s to 2000°/s;
	float 		scaleMagFactor  = 0.1465;

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
        
	    mag[i][0] = (dataBuffer_[13 +(p_.byteIMU*i)]<<8 | dataBuffer_[14 +(p_.byteIMU*i)]);
        mag[i][1] = (dataBuffer_[15 +(p_.byteIMU*i)]<<8 | dataBuffer_[16 +(p_.byteIMU*i)]);
        mag[i][2] = (dataBuffer_[17 +(p_.byteIMU*i)]<<8 | dataBuffer_[18+(p_.byteIMU*i)]);

        Acc_(i,0) = (double)  acc[i][0] * scaleAccFactor;
        Acc_(i,1) = (double)  acc[i][1] * scaleAccFactor;
        Acc_(i,2) = (double)  acc[i][2] * scaleAccFactor;

        Gyro_(i,0) = (double) gyro[i][0] * scaleGyroFactor;
        Gyro_(i,1) = (double) gyro[i][1] * scaleGyroFactor;
        Gyro_(i,2) = (double) gyro[i][2] * scaleGyroFactor;

        Gyro_(i,0) -= Gyro_Bias_(i,0);
        Gyro_(i,1) -= Gyro_Bias_(i,1);
        Gyro_(i,2) -= Gyro_Bias_(i,2);

		Eigen::Vector3d Mag_tmp;
        Mag_tmp(0) = (double)  mag[i][0] * scaleMagFactor * MagCal_(i,0);
        Mag_tmp(1) = (double)  mag[i][1] * scaleMagFactor * MagCal_(i,1);
        Mag_tmp(2) = (double)  mag[i][2] * scaleMagFactor * MagCal_(i,2);

        // Mag_tmp = Mag_tmp/Mag_tmp.norm();

        // Mag_tmp = rotZ(-90) * rotX(180) * Mag_tmp;        
		Mag_(i,0) = -Mag_tmp(1);
        Mag_(i,1) = -Mag_tmp(0);
        Mag_(i,2) = Mag_tmp(2);

        
        if(_DEBUG_REAL_DATA_)
        {
        	// std::cout << i <<"  acc:\t\t" << Acc_(i,0) << "\t" << Acc_(i,1) << "\t" << Acc_(i,2)<<"\r\n"; 
        	// std::cout << " " <<"  gyro:\t" << Gyro_(i,0) << "\t" << Gyro_(i,1) << "\t" << Gyro_(i,2)<<"\r\n";
        	// std::cout << i << " " <<"  mag:\t" << Mag_(i,0) << "\t" << Mag_(i,1) << "\t" << Mag_(i,2)<<"\r\n";
        }
	}

    magCorrector();

	if(_DEBUG_REAL_DATA_)
	{
		std::cout <<"\n\n";
	}
}





// =============================================================================================
//																				 MadgwickGeneral
// =============================================================================================
void IMUGL::Magdwick(int P, int N)
{
	//come N vede P;  N è il mio d (wordl), P è il mio s (sensor)
	// ad esempio N=imu1, P=imu0, quindi come la IMU 1 vede la IMU 0 

	// float recipNorm;
	// float qDot1, qDot2, qDot3, qDot4;
	float q1, q2 ,q3 ,q4;
    
    float dx, dy, dz;
    float sx, sy, sz;

    Eigen::Vector3d aP, aN, mP, mN;
    Eigen::Vector4d gP, gN, g, qL;

    Eigen::Vector3d fa, fm;
    Eigen::VectorXd f(6);
    Eigen::MatrixXd Ja(3,4); 
    Eigen::MatrixXd Jm(3,4); 
    Eigen::MatrixXd Jtot(6,4); 
    Eigen::Vector4d  qdot;

   	Eigen::Vector4d Napla;

	/************************
	*	    READ  DATA		*
	*************************/
	aP(0)  = Acc_(P,0);  
	aP(1)  = Acc_(P,1);  
	aP(2)  = Acc_(P,2);

	aN(0)  = Acc_(N,0);  
	aN(1)  = Acc_(N,1);  
	aN(2)  = Acc_(N,2);
	
	mP(0)  =  Mag_(P,0);  
	mP(1)  =  Mag_(P,1);  
	mP(2)  =  Mag_(P,2);

	mN(0)  =  Mag_(N,0);  
	mN(1)  =  Mag_(N,1);  
	mN(2)  =  Mag_(N,2);

	mP = mP / mP.norm();
	mN = mN / mN.norm();
 	
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
	
	gP = gP*(M_PI/180);
	gN = gN*(M_PI/180);

	
	//qL local quaternion
	qL << Q_(P,0), Q_(P,1), Q_(P,2), Q_(P,3); 
	
	g = QxQ(ConjQ(qL), QxQ(qL, gP)) - gN;	
	// g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;	


 
	if (P==999)
	{
			// Eigen::Vector4d gdeg;
			// gdeg = g * 180 / M_PI;
		// std::cout << P <<"gdeg :\t\t" << gdeg(0) << "\t\t" << gdeg(1) << "\t" << gdeg(2) << "\t" << gdeg(3)<<"\r\n"; 
		std::cout << P <<"  acc:\t\t" << aP(0) << "\t" << aP(1) << "\t" << aP(2)<<"\r\n"; 
        std::cout << P << " " <<"  mag:\t" << mP(0) << "\t" << mP(1) << "\t" << mP(2) <<"\r\n";
		std::cout << N <<"  acc:\t\t" << aN(0) << "\t" << aN(1) << "\t" << aN(2)<<"\r\n"; 
       	std::cout << N << " " <<"  mag:\t" << mN(0) << "\t" << mN(1) << "\t" << mN(2)<<"\r\n";
	}


	q1 = qL(0);  
	q2 = qL(1); 
	q3 = qL(2); 
	q4 = qL(3);

	/****************************
	*	    ACC JACOBIAN		*
	*****************************/
	dx = aN(0); 
	dy = aN(1); 
	dz = aN(2); 
	
	sx = aP(0); 
	sy = aP(1); 
	sz = aP(2); 
	
	fa(0) =  2*dx*(0.5 -q3*q3 -q4*q4) + 2*dy*(q1*q4 + q2*q3) + 2*dz*(q2*q4-q1*q3) - sx; 
	fa(1) =  2*dx*(q2*q3 -q1*q4) + 2*dy*(0.5 - q2*q2 - q4*q4) + 2*dz*(q1*q2 + q3*q4) - sy;
	fa(2) =  2*dx*(q1*q3 -q2*q4) + 2*dy*(q3*q4 - q1*q2) + 2*dz*(0.5 - q2*q2 -q3*q3) - sz; 
	
	Ja << 2*dy*q4-2*dz*q3,    2*dy*q3+2*dz*q4 ,        -4*dx*q3+2*dy*q2-2*dz*q1,  -4*dx*q4+2*dy*q1+2*dz*q2,
		  -2*dx*q4+2*dz*q2,   2*dx*q3-4*dy*q2+2*dz*q1, 2*dx*q2+2*dz*q4,           -2*dx*q1-4*dy*q4+2*dz*q3,
		  2*dx*q3-2*dy*q2,    2*dx*q4-2*dy*q1-4*dz*q2, 2*dx*q1+2*dy*q4-4*dz*q3,   2*dx*q2+2*dy*q3;

  	// Napla = Ja.transpose() * fa;

	/****************************
	*	    MAG JACOBIAN		*
	*****************************/
	dx = mN(0); 
	dy = mN(1); 
	dz = mN(2); 
	
	sx = mP(0); 
	sy = mP(1); 
	sz = mP(2); 
   
	fm(0) =  2*dx*(0.5 -q3*q3 -q4*q4) + 2*dy*(q1*q4 + q2*q3) + 2*dz*(q2*q4-q1*q3) - sx; 
	fm(1) =  2*dx*(q2*q3 -q1*q4) + 2*dy*(0.5 - q2*q2 - q4*q4) + 2*dz*(q1*q2 + q3*q4) - sy;
	fm(2) =  2*dx*(q1*q3 -q2*q4) + 2*dy*(q3*q4 - q1*q2) + 2*dz*(0.5 - q2*q2 -q3*q3) - sz; 
	 

	Jm << 2*dy*q4-2*dz*q3,    2*dy*q3+2*dz*q4 ,        -4*dx*q3+2*dy*q2-2*dz*q1,  -4*dx*q4+2*dy*q1+2*dz*q2,
		  -2*dx*q4+2*dz*q2,   2*dx*q3-4*dy*q2+2*dz*q1, 2*dx*q2+2*dz*q4,           -2*dx*q1-4*dy*q4+2*dz*q3,
		  2*dx*q3-2*dy*q2,    2*dx*q4-2*dy*q1-4*dz*q2, 2*dx*q1+2*dy*q4-4*dz*q3,   2*dx*q2+2*dy*q3;

  	// Napla = Jm.transpose() * fm;

	/****************************
	*	    TOT JACOBIAN		*
	*****************************/
	fm *= 0; 
	f << fa(0), fa(1), fa(2), fm(0),fm(1),fm(2);		
	Jtot << Ja(0,0),Ja(0,1),Ja(0,2),Ja(0,3),
			Ja(1,0),Ja(1,1),Ja(1,2),Ja(1,3),
			Ja(2,0),Ja(2,1),Ja(2,2),Ja(2,3),
			Jm(0,0),Jm(0,1),Jm(0,2),Jm(0,3),
			Jm(1,0),Jm(1,1),Jm(1,2),Jm(1,3),
			Jm(2,0),Jm(2,1),Jm(2,2),Jm(2,3);

	Napla = Jtot.transpose() * f;


	/********************
	*	    UPDATE 		*
	*********************/
	// qdot = 0.5*QxQ( qL,g ) - ( z_.betaVector(P)*Napla ); 
	if (Napla.norm() > 1) 
		qdot = 0.5 * QxQ(qL,g) - (z_.betaVector(P) *  (Napla/Napla.norm()));
	else 
		qdot = 0.5 * QxQ(qL,g) - (z_.betaVector(P) *  Napla);
	
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
					Magdwick(IMUchains_[ii][jj],IMUchains_[ii][jj+1]); 
				}
			}
					

			// Record odl Gyro value
			Gyro_Old_ = Gyro_;
			std::cout<<"step init world: "<< k+1 << "\r\n";
			Q_off_= Q_;
		
			usleep(1000);		
		}	
		
		// Compute angles from quaternions
		Q_joint_ = Q_;
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

	boost::posix_time::time_duration relTime;
	boost::posix_time::ptime firstDataTime, absTime;
	absTime = boost::posix_time::microsec_clock::local_time();


	out.resize(p_.nIMU, 3);
	// Read Data From PSoC
    readPSoC();
	firstDataTime = boost::posix_time::microsec_clock::local_time();

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
			Magdwick(IMUchains_[ii][jj],IMUchains_[ii][jj+1]); 
		}
	}

    
	// record Acc data every 100 samples
    Acc_Old_ = Acc_;
    Gyro_Old_ = Gyro_;

    offsetCorrector('s');
    Quat2AngleTot();


    out = IMUs_Angles_;

	relTime = absTime - firstDataTime; 
	// std::cout << "TIME: " << relTime << std::endl;


	
}






// ============================================================================================= 
//																				  printIMUangles
// ============================================================================================= 
void IMUGL::printIMUangles()
{	
	int k = 0;
	std::cout<<"\n";	
	for (int i = 0; i < (int)IMUchains_.size(); i++ )
	{
		for (int j=0; j < (int)IMUchains_[i].size() - 1; j++)
		{
			std::cout << "a " <<IMUchains_[i][j] << " " <<IMUchains_[i][j+1] << ":\t";
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
			Acc_Faliure_(k) = 0; 
			// std::cout<<"\r\nbad communication IMU: "<< k << "\r\n";
		} 
		else 
			Acc_Faliure_(k) = 1;
		
		Current_Acc(0) = Acc_(k,0);
    	Current_Acc(1) = Acc_(k,1);
    	Current_Acc(2) = Acc_(k,2);
    	if (Current_Acc.norm() > 1.15) 
    	{
      		Acc_(k,0) = Acc_Old_(k,0);
      		Acc_(k,1) = Acc_Old_(k,1);
      		Acc_(k,2) = Acc_Old_(k,2);
			
			// Gyro_(k,0) = 0; 
			// Gyro_(k,1) = 0; 
			// Gyro_(k,2) = 0; 
			// Gyro_ *=0;

			z_.betaVector(k) = 0.1;
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

}


void IMUGL::Update()
{
	float gx = Gyro_(0,0);
	float gy = Gyro_(0,1);
	float gz = Gyro_(0,2);

	float ax = -Acc_(0,0);
	float ay = -Acc_(0,1);
	float az = -Acc_(0,2);

	float mx = Mag_(0,0);
	float my = Mag_(0,1);
	float mz = Mag_(0,2);


    float q1 = Quaternion(0), q2 = Quaternion(1), q3 = Quaternion(2), q4 = Quaternion(3);   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2 * q1;
    float _2q2 = 2 * q2;
    float _2q3 = 2 * q3;
    float _2q4 = 2 * q4;
    float _2q1q3 = 2 * q1 * q3;
    float _2q3q4 = 2 * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = (float)sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Normalise magnetometer measurement
    norm = (float)sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2 * q1 * mx;
    _2q1my = 2 * q1 * my;
    _2q1mz = 2 * q1 * mz;
    _2q2mx = 2 * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = (float)sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2 * _2bx;
    _4bz = 2 * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    norm = 1 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    float Beta = 2;
    float SamplePeriod = 0.016;


    // Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;


    // Integrate to yield quaternion
    q1 += (float) qDot1 * SamplePeriod;
    q2 += (float) qDot2 * SamplePeriod;
    q3 += (float) qDot3 * SamplePeriod;
    q4 += (float) qDot4 * SamplePeriod;
    norm = 1 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    Quaternion(0) = q1 * norm;
    Quaternion(1) = q2 * norm;
    Quaternion(2) = q3 * norm;
    Quaternion(3) = q4 * norm;
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
	float x, y, z, w;
	float r21, r11, r31, r22, r23; 
	for (int k=0; k<p_.nIMU; k++) 
	{
 		// IMUs_Angles_(k,0) = atan2(2*Q_joint_(k,1)*Q_joint_(k,2) - 2*Q_joint_(k,0)*Q_joint_(k,3), 2*Q_joint_(k,0)*Q_joint_(k,0) + 2*Q_joint_(k,1)*Q_joint_(k,1)-1); //YAW
	 	// IMUs_Angles_(k,1) = -asin(2*Q_joint_(k,1)*Q_joint_(k,3) + 2*Q_joint_(k,0)*Q_joint_(k,2)); //PITCH
 	 // 	IMUs_Angles_(k,2) = atan2(2*Q_joint_(k,2)*Q_joint_(k,3) - 2*Q_joint_(k,0)*Q_joint_(k,1), 2*Q_joint_(k,0)*Q_joint_(k,0) + 2*Q_joint_(k,3)*Q_joint_(k,3)-1); // ROLL
	    x = Q_joint_(k,1);
	    y = Q_joint_(k,2);
	    z = Q_joint_(k,3);
	    w = Q_joint_(k,0);
	    r11 = 1 - 2*(y*y + z*z);
	    r21 = 2*(x*y + w*z);
	    r22 = 1 - 2*(x*x + z*z);
	    r23 = 2*(y*z - w*x);
	    r31 = 2*(x*z - w*y);

	    //float num = std::abs(w) - std::abs(z);
	    //float den = std::abs(w) + std::abs(z);
	    //if( num/den > 0.2 )
	    if (fabs(w) > fabs(z))
	     {
	     	IMUs_Angles_(k,0) = - atan2(r21, std::sqrt(r11*r11 + r31*r31)); //YAW
			IMUs_Angles_(k,2) = - atan2(-r31, r11); //PITCH
			IMUs_Angles_(k,1) = - atan2(-r23, r22); // ROLL
	    	// yaw singularity
		 	//IMUs_Angles_(k,0) = - asin(2*x*y + 2*z*w); //YAW
			//IMUs_Angles_(k,2) = - atan2(2*x*w - 2*y*z, 1-2*x*x - 2*z*z); //PITCH
			//IMUs_Angles_(k,1) = - atan2(2*y*w - 2*x*z, 1-2*y*y - 2*z*z); // ROLL
	    	
	    	// pitch singularity
			// IMUs_Angles_(k,0) = atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x-1); //YAW
	 		// IMUs_Angles_(k,1) = -asin(2*x*z + 2*w*y); //PITCH
 	 	// 	IMUs_Angles_(k,2) = atan2(2*y*z - 2*w*x, 2*w*w + 2*z*z-1); // ROLL
		//	IMUs_Angles_old_ = IMUs_Angles_;
	     }
	     else
	     {
	     	IMUs_Angles_(k,0) = - atan2(r21, -std::sqrt(r11*r11 + r31*r31)); //YAW
			IMUs_Angles_(k,2) = - atan2(r31, -r11); //PITCH
			IMUs_Angles_(k,1) = - atan2(r23, -r22); // ROLL
	  //   	IMUs_Angles_(k,0) = IMUs_Angles_old_(k,0);
			// IMUs_Angles_(k,2) = IMUs_Angles_old_(k,2);
			// IMUs_Angles_(k,1) = IMUs_Angles_old_(k,1);
	     }


		if(k==0)
		{	
		    // std::cout<<"num/den   " << num/den << std::endl;
			std::cout << "w: " << w << "\t\tx: " << x << "\t\ty: " << y << "\t\tz: " << z << std::endl;
   		}
 //   	IMUs_Angles_(k,0) = std::atan2(2.0f * (w * z + x * y),1.0f - 2.0f * (y * y + z * z)); //YAW
	// IMUs_Angles_(k,1) = std::asin(std::max(-1.0f, std::min(1.0f, 2.0f * (w * y - z * x))));
 //   	IMUs_Angles_(k,2) = std::atan2(2.0f*(w*x +y*z),1.0f - 2.0f*(x * x + y * y));
   }
}


void IMUGL::offsetCorrector(char choice)
{
  if (choice == 'n')
  {
    Q_joint_ = Q_;
  }
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



