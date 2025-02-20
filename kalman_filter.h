#include <stdint.h>
#include "Eigen/Dense"

namespace KF 
{
    template <int32_t DIM_X, int32_t DIM_Z>
    class KalmanFilter {
    public:
    KalmanFilter()=default;
     
    void prediction(Eigen::Matrix<double, DIM_X, DIM_X> &A ,Eigen::Matrix<double, DIM_X, DIM_X> Q)
    {
        m_vectorX = A * m_vectorX;
        m_matrixP = A * m_matrixP * A.transpose() + Q;

    }

    void correction(const Eigen::Vector<double, DIM_Z> &vector_z, const  Eigen::Matrix<double, DIM_Z, DIM_X> &H, Eigen::Matrix<double, DIM_Z, DIM_Z> &R)
    {
        auto vector_y = vector_z -(H * m_vectorX);
        auto S = H * m_matrixP * H.transpose() + R;
        auto K = m_matrixP * H.transpose() * S.inverse(); 
    }
    private:
     /// 
     /// @brief state vector
     ///
     Eigen::Vector<double, DIM_X> m_vectorX;// state vector
     /// 
     /// @brief state covariecne matrix
     ///
     Eigen::Matrix<double, DIM_X, DIM_X> m_matrixP; //Covarience matrix
    };
}