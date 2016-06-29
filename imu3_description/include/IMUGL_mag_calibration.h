/*
    IMUGL_cal - IMU GLOVE LIBRARY
    
    Copyright (C) 2016  Emanuele Luberto (emanuele.luberto@gmail.com)
    
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
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/serial_port.hpp> 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>


#define _DEBUG_DATA_BUFFER_ 0
#define _DEBUG_REAL_DATA_   1
#define _CALIBRATION_STEPS_ 1000


// sgn function
#define sgn(s) (s>=0.0?1:-1)


/* Variables useful to PSoC communication */
struct psocVariables
{
    // PSoC port
    char*   port; 
    // baudRate 
    int     baudRate;
    // # of IMU connected
    int     nIMU;
    // buffer size for one IMU
    int     byteIMU;   
};

/* Variables useful to Madgwick filter */
struct filterVariables
{
    float thGyro;
    float beta;       
    float sampleFreq; 
    Eigen::VectorXd betaVector;
};


class IMUGL_cal
{
public:
    /*          Contructor         */
    IMUGL_cal();

    /*          Destructor         */
    ~IMUGL_cal();
    
    // public variables
    Eigen::MatrixXd Acc_;
    Eigen::MatrixXd Acc_Old_;
    Eigen::MatrixXd Gyro_;
    Eigen::MatrixXd Gyro_Old_;
    Eigen::MatrixXd Gyro_Bias_;
    Eigen::MatrixXd Mag_;
    Eigen::MatrixXd MagCal_;

   

    //================================================================     InitPSoC
    void initPSoC();
    psocVariables p_;
    
    //================================================================     StopPSoC
    void stopPSoC();
    
    //================================================================     ReadPSoC
    void readPSoC();

    //================================================================     Compass Calibration 
    void compassCalibration(std::string path);    
    int calibration_step_;


    void pInv_mia(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_pinv, bool damped = true) const;

private:
    boost::asio::serial_port* serialPort_;
    boost::asio::io_service ioService_;
    int sizeBuffer_;
    uint8_t* dataBuffer_; 


    //================================================================     Compute Mag Factor
    void computeMagFactors(Eigen::MatrixXd M, int d);
    Eigen::MatrixXd corrector_Factors_;


   

    //================================================================     Useful Functions 
    Eigen::Matrix3d rotX( float x ) const;
    Eigen::Matrix3d rotY( float y ) const;
    Eigen::Matrix3d rotZ( float z ) const;
    Eigen::Matrix3d rot( float x, float y, float z ) const;
    Eigen::Matrix3d skew( Eigen::Vector3d a ) const;

};

































