#include "kalman_filter.h"
#include "Eigen/Dense"
#include <iostream>

int main(int argc, char const *argv[])
{
    constexpr int32_t DIM_X = 2;
    constexpr int32_t DIM_Z = 1;

    KF::KalmanFilter<2, 1> kalmanFilter;
    kalmanFilter.state() << 0.0, 1.0;
    kalmanFilter.matP() << 1.0, 0.0,
                           0.0, 1.0;
    
    Eigen::Matrix<double, DIM_X, DIM_X> matF;
    matF<<1.0,1.0,
         0.0,1.0;
    
    Eigen::Matrix<double, DIM_X, DIM_X> matQ; //process noise covarience
    matQ<<0.5,0.0,
          0.0,0.5;
    
    kalmanFilter.prediction(matF, matQ);

    std::cout<<"After prediction"<<std::endl;
    std::cout<<"x value: "<<kalmanFilter.state()<<std::endl;
    std::cout<<"P value: "<<kalmanFilter.matP()<<std::endl;

    Eigen::Matrix<double, DIM_Z, DIM_Z> matR;
    matR<<0.1;
    Eigen::Vector<double, DIM_Z> vecZ;
    vecZ<<1.2;
    Eigen::Matrix<double, DIM_Z, DIM_X> matH;
    matH<<1.0,0.0;
    kalmanFilter.correction(vecZ, matH, matR);
    
    std::cout<<"After correction"<<std::endl;
    std::cout<<"x value: "<<kalmanFilter.state()<<std::endl;
    std::cout<<"P value: "<<kalmanFilter.matP()<<std::endl;
    return 0;
}