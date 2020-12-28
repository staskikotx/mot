#pragma once

#include "Eigen/Dense"
#include <iostream>

using namespace std;

class KF
{
public:
    static const int NUM_VARS = 4;
    static const int iX = 0;
    static const int iY = 1;
    static const int iVX = 2;
    static const int iVY = 3;
    static constexpr double measVarianceX = 0.01;
    static constexpr double measVarianceY = 0.1;

    using Vector = Eigen::Matrix<double, NUM_VARS, 1>;
    using Matrix = Eigen::Matrix<double, NUM_VARS, NUM_VARS>;
    
private:
    
    
    Vector State;
    Matrix CovariationMatrix;

    const double m_accelVariance;

public:

    KF(double initialX = -450 , double initialY = 4500 , 
       double initialVX = 0 , double initialVY = 0, 
       double accelVariance = 1, double initialUncertainty = 100) 
            : m_accelVariance(accelVariance)
    {
        State(iX) = initialX;
        State(iY) = initialY;
        State(iVX) = initialVX;
        State(iVY) = initialVY;

        CovariationMatrix.setIdentity();
        
        CovariationMatrix = CovariationMatrix * initialUncertainty;
    }

    void Predict(double dt)
    {
        Matrix F;
        F.setIdentity();
        F(iX, iVX) = dt;
        F(iY, iVY) = dt;
        
        const Vector newX = F * State;

        Vector G;
        
        G(iX) = 0.5 * dt * dt;
        G(iY) = 0.5 * dt * dt;
        G(iVX) = dt;
        G(iVY) = dt;

        const Matrix newP = F * CovariationMatrix * F.transpose() + G * G.transpose() * m_accelVariance;

        CovariationMatrix = newP;
        State = newX;
    }

    void Update(double measValueX, double measValueY)
    {        
        
        Eigen::Matrix<double, 2, 4> H;
        H.setZero();
        H(iX, iX) = 1;
        H(iY, iY) = 1;

        Eigen::Matrix<double, 2, 2> R;
        R.setZero();
        R(iX, iX) = measVarianceX;
        R(iY, iY) = measVarianceY;
        
        Eigen::Matrix<double, 2, 1> z;
        z(iX) = measValueX;
        z(iY) = measValueY;
        
        
        Eigen::Matrix<double, 2, 1> y = z - H * State;
         
        
        const Eigen::Matrix<double, 2, 2>  S = H * CovariationMatrix * H.transpose() + R;        

        const Eigen::Matrix<double, 4, 2> K = CovariationMatrix * H.transpose() * 1.0 * S.
        inverse();
                   
        Vector newX = State + K * y;
                                
        Matrix newP = (Matrix::Identity() - K * H) * CovariationMatrix;

        CovariationMatrix = newP;
        State = newX;
    }

    Vector GetState() const
    {
        return State;
    }

};
