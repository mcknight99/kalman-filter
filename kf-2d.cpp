#include "kf-2d.h"
#include <iostream>

// orthogonalization/diagonalization
// symmetric matrices
// orthogonal diagonalization
// least squares for error

// Good videos from ilectureonline / michel van biezen youtube
// special topics kalman filter
// 7,8,9,10

KF2D::KF2D() {} // default constructor, don't want a static KF

// Initialize Kalman Filter matrices and state vector
void KF2D::InitializeKalmanFilter(const MeasurementVector &measurement)
{

     // Initialize P, A, H, R, Q with appropriate values //NEEDS TO BE DONE: P, R, Q
     // dt will NOT be consistent between updates, so they need to be replaced every update
     // This depends on the specifics of the system and sensor characteristics

     // function to update covariance matrices (PQR)

     // Michel van Biezen Kalman Filter #19, 20, 21, 22, 23

     // P and Q will be one day automatically recalibrating for any sensor set

     // State covariance matrix (error in the estimate)
     // If P->0, then measurement updates are mostly ignored
     //  P is symmetric, no need to col maj (for now)
     P = {{0.3, 0.3, 0.3},
          {0.3, 0.3, 0.3},
          {0.3, 0.3, 0.3}};

     // P = {{0.99, 0.9, 0.9},
     //{0.9, 0.9, 0.8,
     //{0.9, 0.8, 0.1}};

     /*great results on test launch 1 (577.288 apogee)
     P = {{0.99, 0.9, 0.9},
     {0.9, 0.9, 0.7},
     {0.9, 0.7, 0.1}};  */

     // Process noise covariance matrix (inherent error in the sensors)
     //(keeps the state covariance matrix, P, from becoming too small or going to 0)
     Q = {{0.3, 0.3, 0.3},
          {0.3, 0.3, 0.3},
          {0.3, 0.3, 0.3}};

     // Q = {{0.9, 0.5, 0.1},
     //{0.5, 0.2, 0.15},
     //{0.1, 0.15, 0.3}};

     /*great results on test launch 1 (577.288 apogee)
     Q = {{0.9, 0.5, 0.2},
     {0.5, 0.4, 0.15},
     {0.2, 0.15, 0.3}};*/

     // Measurement coviariance matrix (error in the measurement)
     //  R is symmetric if and only if covariances are equal, so be careful
     // If R->0, then K->1 (trust bias to measurement update)
     // If R->INF, then K->0 (trust bias to predicted state)
     R = {{0.5, 0.0},  // ay
          {0.0, 0.5}}; // y
     // will be 0.1,0,0 ; 0,0.1,0 ; 0,0,0.1 in 3d, or whatever R float instead of 0.1

     // rocket is 2.5676298406m

     // Initialize the state vector with initial values
     x_hat = {measurement[0], 0, measurement[1]}; // Initial posititon, velocity, acceleration (y, vy, ay)
     // should eventually be [0] [1] [2] with a pitot

     /*
     //State to measurement matrix
     //basically picks which state variables refer to which measurements
     //this is what it SHOULD look like written out by hand, but linalg is col maj
        H = {{1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0}};

     //And this is a 3d H (which is symmetric and doesn't matter code vs hand)
       H = 1 0 0
           0 1 0
           0 0 1
            */
     H = {{1, 0}, {0, 0}, {0, 1}};
     Update(measurement, 0);
}

// Predict the next state using the process model
void KF2D::Predict()
{
     // Predict the state using the state transition matrix A and the previous state estimate x_hat
     x_hat = mul(A, x_hat) + (B * uk); //+wk implementation for manual adjustments?

     // Update the error covariance matrix P using the process noise covariance Q
     P = mul(mul(A, P), transpose(A)) + Q;
}

// Update the state estimate based on a new measurement
// Might need another param for the dt for the transformation matrix
// DEFINITELY need another param for the dt for the transformation matrix, UNLESS it can be done locally
// great news it could be done locally
// In either case, it needs to be update in state matrices using dt
void KF2D::Update(const MeasurementVector &measurement, float delta_time) //, float delta_time
{
     // long long now = getNow();
     // float delta_time = (float)(now-lastTime);
     // lastTime = now;
     // delta_time/=1E9; //convert ns to s
     // uk = measurement[1]; // should be measurement[2] when we read y, vy, ay
     // actually got removed when gravity gets shifted in the measurement

     /*
     auto thisTime = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
     float delta_time = (thisTime - lastTime) / 1E9; // convert nanoseconds to seconds
     lastTime = thisTime;
     */
     // std::cout << "dt=" << delta_time << "\t";

     /* //this is what it SHOULD look like written out by hand, but linalg.h is col maj
          A = {{1,    delta_time,    0.5 * delta_time * delta_time},
               {0,    1,             delta_time},
               {0,    0,             1}};
               */
     A = {{1, 0, 0},
          {delta_time, 1, 0},
          {(float)(0.5 * delta_time * delta_time), delta_time, 1}};

     // B is really just a vector in our example because it is being added to the state vector and multiplied by the scalar control variable
     B = {(float)(0.5 * delta_time * delta_time), delta_time, 1};

     // Calculate the Kalman gain
     mat<float, 3, 2> K = mul(mul(P, transpose(H)), inverse(mul(mul(H, P), transpose(H)) + R)); // n,m

     // Update the state estimate based on the measurement and Kalman gain
     x_hat = x_hat + mul(K, (measurement - mul(H, x_hat)));

     // Update the error covariance matrix P
     float3x3 I = identity; // identity is a built in linalg expression
     P = mul((I - mul(K, H)), P);

     Predict();
}

KF2D::StateVector KF2D::getPrediction()
{
     return x_hat;
}

long long KF2D::getNow()
{
     // Get the current time point from the high-resolution clock
     auto now = std::chrono::high_resolution_clock::now();

     // Convert the time point to nanoseconds since the epoch
     std::chrono::nanoseconds::rep now_in_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
     return now_in_nanoseconds;
}

long long KF2D::getLastTime()
{
     return lastTime;
}

void KF2D::UpdateCovariance(const MeasurementVector &measurement)
{
    measurement_history.push_back(measurement);
    state_history.push_back(x_hat);

    // Logic to update Q and R based on historical data
    if (measurement_history.size() > 1)
    {
        float2 measurement_diff = measurement - measurement_history[measurement_history.size() - 2];
        R = R + outerprod(measurement_diff, measurement_diff);
        R = R / (measurement_history.size() - 1);
    }

    if (state_history.size() > 1)
    {
        float3 state_diff = x_hat - state_history[state_history.size() - 2];
        Q = Q + outerprod(state_diff, state_diff);
        Q = Q / (state_history.size() - 1);
    }
    
}

void KF2D::clearHistory()
{
    measurement_history.clear();
    state_history.clear();
}