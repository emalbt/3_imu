/*
    IMUGL - IMU GLOVE LIBRARY
    
    Copyright (C) 2016  Emanuele Luberto (emanuele.luberto@gmail.com)
    
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

// # step for compute offset angles
#define _OFFSET_STEP_      90


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


class IMUGL
{
public:
    /*          Contructor         */
    IMUGL();

    /*          Destructor         */
    ~IMUGL();
    
    // public variables
    Eigen::MatrixXd Acc_;
    Eigen::MatrixXd Acc_Old_;
    Eigen::MatrixXd Gyro_;
    Eigen::MatrixXd Gyro_Old_;
    Eigen::MatrixXd Gyro_Bias_;

    Eigen::MatrixXd IMUs_Angles_;
    Eigen::MatrixXd Q_; 
    Eigen::MatrixXd Q_off_; 
    Eigen::MatrixXd Q_joint_;

    //================================================================     InitPSoC
    void initPSoC();
    psocVariables p_;
    
    //================================================================     StopPSoC
    void stopPSoC();
    
    //================================================================     ReadPSoC
    void readPSoC();

    //================================================================     MadgwickGeneral
    void MadgwickGeneral( int P, int N );

    //================================================================     InitialOffset
    void initialOffset();
    filterVariables z_;

    //================================================================     ComputeAngles
    void computeAngles(Eigen::MatrixXd& out);
    int count_;

    //================================================================     CheckChains
    void checkChains();

    //================================================================     AddChain
    void addChain( std::vector<int> a );
    std::vector<std::vector<int> >   IMUchains_;

    //================================================================     PrintIMUangles
    void printIMUangles();
    
    //================================================================     Useful Functions 
    Eigen::Matrix3d rotX( float x );
    Eigen::Matrix3d rotY( float y );
    Eigen::Matrix3d rotZ( float z );
    Eigen::Matrix3d rot( float x, float y, float z );
    Eigen::Matrix3d skew( Eigen::Vector3d a );
    Eigen::Matrix3d Quat2Rot( Eigen::Vector4d Q_in );
    Eigen::Vector4d ConjQ( Eigen::Vector4d Q_in );
    Eigen::Vector4d QxQ( Eigen::Vector4d Q_1, Eigen::Vector4d Q_2 );
    Eigen::Vector4d Rot2Quat( Eigen::Matrix3d R_in );
    Eigen::Vector3d Quat2Angle( Eigen::Vector4d Q_in );
    Eigen::Vector3d Rot2Angle( Eigen::Matrix3d R_in );
    void Quat2AngleTot();
    void offsetCorrector(char c);
    void AngleTot2Quat();
    

private:

    boost::asio::serial_port* serialPort_;
    boost::asio::io_service ioService_;
    int sizeBuffer_;
    uint8_t* dataBuffer_; 


    //================================================================     CheckIMU 
    void checkIMU();
    Eigen::VectorXd Acc_Faliure_;
    Eigen::VectorXd count_IMU_Failure_;
    

  
};

































