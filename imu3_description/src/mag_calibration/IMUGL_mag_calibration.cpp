/*
	IMUGL_cal - IMU GLOVE LIBRARY
	
    Copyright (C) 2016	Emanuele Luberto (emanuele.luberto@gmail.com)
	
	Author affiliation:
		Emanuele Luberto - Research Center “E.Piaggio”,School of Engineering,University of Pisa 
							from 01-april-2016 to current


    IMUGL_cal is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or 
	any later version.

    IMUGL_cal is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FORz A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SHOG. If not, see <http://www.gnu.org/licenses/>.
*/

#include "IMUGL_mag_calibration.h"
#include <boost/date_time/posix_time/posix_time.hpp>


// ============================================================================ Constructor#2
IMUGL_cal::IMUGL_cal()
// IMUGL_cal::IMUGL_cal(psocVariables p_constructor, bool flag)
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
	
	std::cout<<"\nWelcome\n\tIMUGL_cal "<<ssVersion.str()<<"\n";
}


// =============================================================================================
//																					 Desctructor
// =============================================================================================
IMUGL_cal::~IMUGL_cal()
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
void IMUGL_cal::initPSoC()
{

	// init public variables
	Acc_.resize(p_.nIMU, 3);
	Acc_Old_.resize(p_.nIMU,3);
	Gyro_.resize(p_.nIMU, 3);
	Gyro_Old_.resize(p_.nIMU,3);
	Gyro_Bias_.resize(p_.nIMU,3);
	Gyro_Bias_.setZero();
	
	Mag_.resize(p_.nIMU,3);
	MagCal_.resize(p_.nIMU,3);
	corrector_Factors_.resize(p_.nIMU,6);




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

	std::cout<< "\r\nCOMMUNICATION ESTABLISHED WITH THE PSOC5 \n";
	
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
void IMUGL_cal::stopPSoC()
{
	std::cout<< "\r\n\nSTOP COMMUNICATION\n";
	serialPort_->close();
}


// =============================================================================================
//																						ReadPsoC
// =============================================================================================
void IMUGL_cal::readPSoC()
{
	// IMU glove
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
	float 		scaleMagFactor  = 0.1465; // " *8 " = from 250°/s to 2000°/s;

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

        // Mag_ = rotZ(-90) * rotX(180) * Mag_tmp;        
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

	if(_DEBUG_REAL_DATA_)
	{
		std::cout <<"\n\n";
	}
}



// =============================================================================================
//																			  compassCalibration
// =============================================================================================
void IMUGL_cal::compassCalibration(std::string path)
{

	std::vector<Eigen::Matrix<double, _CALIBRATION_STEPS_, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, _CALIBRATION_STEPS_, 3>> > M;
	M.resize(p_.nIMU);

	for (int i = 0; i< _CALIBRATION_STEPS_; i++)
	{
		readPSoC();
		for(int j=0; j<p_.nIMU; j++)
		{
			M[j](i,0) = Mag_(j,0);
			M[j](i,1) = Mag_(j,1);
			M[j](i,2) = Mag_(j,2);
        	std::cout << i << " " <<"  mag:\t" << Mag_(j,0) << "\t" << Mag_(j,1) << "\t" << Mag_(j,2)<<"\r\n";

		}
		std::cout << "+ calibration step: " << i << std::endl;
	}

	for (int i=0; i<p_.nIMU; i++)
	{
		computeMagFactors(M[i], i);
	}


	Eigen::MatrixXd CF(p_.nIMU,6);
	CF = corrector_Factors_;

	FILE *MCFfile = fopen( (path + "Matrix_Corrector_Factors.txt").c_str(),"w");
	for(int k=0; k<p_.nIMU; k++ ){
			fprintf(MCFfile, "%f\t%f\t%f\t%f\t%f\t%f\n", CF(k,0),CF(k,1),CF(k,2),CF(k,3),CF(k,4),CF(k,5) );
	}
	fclose(MCFfile);

}


// =============================================================================================
//																			   computeMagFactors
// =============================================================================================
void IMUGL_cal::computeMagFactors(Eigen::MatrixXd Data_Mag, int n){
	int N = _CALIBRATION_STEPS_;
	Eigen::VectorXd x(N),x2(N), y(N),y2(N), z(N), z2(N), F(6);
	Eigen::MatrixXd D(N,6);

	for(int k=0; k<N; k++)
	{
		x(k)  = Data_Mag(k,0);
		y(k)  = Data_Mag(k,1);
		z(k)  = Data_Mag(k,2);
		x2(k) = Data_Mag(k,0)*Data_Mag(k,0);
		y2(k) = Data_Mag(k,1)*Data_Mag(k,1);
		z2(k) = Data_Mag(k,2)*Data_Mag(k,2);
	}
	
	for(int k=0; k<N; k++)
	{
		D(k,0) = x(k);
		D(k,1) = y(k);
		D(k,2) = z(k);
		D(k,3) = -y2(k);
		D(k,4) = -z2(k);
		D(k,5) = 1;
	}
	
	Eigen::MatrixXd D_inv;
	pInv_mia(D, D_inv, true); 

	F = D_inv * x2;

	

	//offset
	corrector_Factors_(n,0) = F(0)/2;
	corrector_Factors_(n,1) = F(1)/(2*F(3));
	corrector_Factors_(n,2) = F(2)/(2*F(4));
	
	//gain
	float A = F(5) + (F(0)/2)*(F(0)/2)   + F(3)*(F(1)/(2*F(3)))*(F(1)/(2*F(3))) + F(4)*(F(2)/(2*F(4)))*(F(2)/(2*F(4)));

	corrector_Factors_(n,3) = 1/sqrt(std::abs(A));
	corrector_Factors_(n,4) = 1/sqrt(std::abs(A/F(3)));
	corrector_Factors_(n,5) = 1/sqrt(std::abs(A/F(4))); 	
}






// ============================================================================================= 
//																				Useful Functions
// =============================================================================================
void IMUGL_cal::pInv_mia(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_pinv, bool damped) const
{	
	double lambda_ = damped?1E-9:0.0;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	Eigen::MatrixXd S = M;	// copying the dimensions of M_, its content is not needed.
	S.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv = Eigen::MatrixXd(svd.matrixV()*S.transpose()*svd.matrixU().transpose());

}


Eigen::Matrix3d IMUGL_cal::rotX(float x)   const
{
	Eigen::Matrix3d Rx;
	x = x *(M_PI/180); // From deg to Rad
	Rx << 1 , 0, 0,
		  0, cos(x), -sin(x),
		  0, sin(x), cos(x);
	return Rx;
}
Eigen::Matrix3d IMUGL_cal::rotY(float y)  const
{
	Eigen::Matrix3d Ry;
	y = y *(M_PI/180); // From deg to Rad
	Ry << cos(y) , 0, sin(y),
		  0,       1,   0,
		  -sin(y), 0, cos(y);
	return Ry;
}

Eigen::Matrix3d IMUGL_cal::rotZ(float z) const
{
	Eigen::Matrix3d Rz;
	z = z *(M_PI/180); // From deg to Rad
	Rz << cos(z), -sin(z), 0,
		  sin(z), cos(z),  0,
		  0,        0,     1;
	return Rz;
}

Eigen::Matrix3d IMUGL_cal::rot(float x, float y, float z) const
{
	Eigen::Matrix3d Rx, Ry, Rz, Rf;
	
	Rx = rotX(x);
	Ry = rotY(y);
	Rz = rotZ(z);

	Rf = Rz*Ry*Rx;
	return Rf;
}
//Skew Matrix
Eigen::Matrix3d IMUGL_cal::skew(Eigen::Vector3d a) const
{
	Eigen::Matrix3d q;
	 q(0,0) =  0;          q(0,1) = -a(2);       q(0,2) = a(1);
	 q(1,0) =  a(2);       q(1,1) =  0;          q(1,2) = -a(0);
	 q(2,0) = -a(1);       q(2,1) =  a(0);       q(2,2) = 0;
		return q;
}
