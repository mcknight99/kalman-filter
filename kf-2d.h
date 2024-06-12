#ifndef KF2D_H
#define KF2D_H

#include "linalg.h"  //matrix declaration is column major
#include <chrono>    //for keeping track of dt between updates //deprecated: FOR NOW. 
using namespace linalg::aliases;
using namespace linalg;
using namespace std::chrono; //deprecated

class KF2D
{
public:
    // Derivables and measurables
    //int n = 3; // Number of state variables, measurable or derivable by equations
    //int m = 2; // Number of measurable variables by sensor //will be 3 with pitot or vin --> replace all 2's with 3's & measurementvector with float3


    // Define the state vector (position, velocity, acceleration) and measurement vector (altitude, vertical velocity, vertical acceleration)
    typedef float3 StateVector;       // state variable x, 3 float vector
    typedef float2 MeasurementVector; // state measurement z, 2 float vector // will be 3 with pitot or vin


    KF2D();                                                            // constructor
    void InitializeKalmanFilter(const MeasurementVector &measurement); // initialize, only needs to run once. can also be used to reset a KF. should be ran at
    void Predict();                                                    // should I also have a delta time predict?? or take average over time dt and use that
    void Update(const MeasurementVector &measurement); //, float delta_time
    StateVector getPrediction();                                       // return the x_hat StateVector
    long long getNow();                                                // return the time in ns at call 
    long long getLastTime();                                           // return the last time the filter was updated in ns

private:
    // Define Kalman Filter matrices (P, A, H, R, Q)
    mat<float, 3, 3> P; // Estimate error covariance //n,n
    mat<float, 3, 3> A; // State transition matrix //n,n
    mat<float, 2, 3> H; // State to measurement matrix //m,n
    mat<float, 2, 2> R; // Measurement noise covariance matrix //m,m
    mat<float, 3, 3> Q; // Process noise covariance matrix //n,n
    float3 B;           // Control Variable transition matrix //ulen,n
    float uk;           // control matrix variable

    float lastTime; //

    StateVector x_hat; // Estimated state

    //debating removing the accel from the state matrix, maybe even adding the derivations and integrations for velocity as inputs and letting the system choose 
    //between state, deriv, integ, and eventually pitot

    // Phone: -144m +-12m (1030.74 hPa)
    // Altimeter: -147m
    // 3m off of phone, +-15m
};

#endif