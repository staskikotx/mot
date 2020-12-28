#pragma once

#include "kf.h"
#include <cmath>
#include <utility>

using namespace std;

class SingleObjectTracker
{
public:
    static constexpr double XWeight = 20.0, YWeight = 1;
    static constexpr double InactiveTimeBeforeRemoval = 1000; //1 sec = 1000 ms
private: 
    KF KalmanFilter;
    int CurrentTimeStamp;
    int LastRegisteredTimeStamp;
    double CurrentX, CurrentY;
    int Id;
    bool Open;
    
public:
    SingleObjectTracker(int timeStamp, double x, double y, int id) :
        CurrentTimeStamp(timeStamp), LastRegisteredTimeStamp(timeStamp),
        CurrentX(x), CurrentY(y), Id(id), Open(true)
    {
        
    }
    
    bool IsOpen() const {
        return Open;
    }
    
    auto GetIdAndPolarVelocity() const {
        KF::Vector state = KalmanFilter.GetState();
        
        int iR = 0, iPhi = 1;
        int iX = 0, iY = 1;
        
        Eigen::Matrix<double, 2, 1> CartesianVelocity;
        CartesianVelocity[iX] = state[KF::iVX];
        CartesianVelocity[iY] = state[KF::iVY];
        double x = state[KF::iX];
        double y = state[KF::iY];
        double rSquared = x*x + y*y;
       
        
        Eigen::Matrix<double, 2, 2>  TransformVelocityToPolarMatrix;
        TransformVelocityToPolarMatrix(iR, iX) = x/sqrt(rSquared);
        TransformVelocityToPolarMatrix(iR, iY) = y/sqrt(rSquared);
        TransformVelocityToPolarMatrix(iPhi, iX) = y/ rSquared ;
        TransformVelocityToPolarMatrix(iPhi, iY) = -x/ rSquared;
        
        Eigen::Matrix<double, 2, 1> 
              PolarVelocity = TransformVelocityToPolarMatrix * CartesianVelocity;
        
        return make_pair(Id, make_pair(PolarVelocity[iR], PolarVelocity[iPhi]));
        //return make_pair(Id, make_pair(CartesianVelocity[iX], CartesianVelocity[iY]));
    }
    
    bool Update(int newTimeStamp) {
        // returns true if the Tracker should be destroyed
        Open = true;
        if (newTimeStamp - LastRegisteredTimeStamp > InactiveTimeBeforeRemoval) {
            return true;
        } else {
            double dt = (newTimeStamp - CurrentTimeStamp) / 1000.0;
            KalmanFilter.Predict(dt);
            return false;
        }
    }
    
    double GetDistance(double objectX, double objectY) {
        return pow(objectX - CurrentX, 2) * XWeight + pow(objectY - CurrentY, 2) * YWeight;
    }

    RegisterEvent(int timeStamp, double x, double y) {        
        KalmanFilter.Update(x, y );
        Open = false;
    }


};
