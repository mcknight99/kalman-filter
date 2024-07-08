#include "apogeeprediction.h"
#include "math.h"
//#include <chrono>
//using namespace std::chrono;
#include <iostream>

//ApogeePrediction::ApogeePrediction(double rocketMass, double dragCoefficient, double crossArea, double targetApogee) : rocketMass(rocketMass), dragCoefficient(dragCoefficient), crossArea(crossArea), targetApogee(targetApogee) {
//    currentVelocity = 0;
//    lastRecTime = 0;
//    predApogee = 0;
//}

void ApogeePrediction::eulerFromQuaternion(double *euler, double x, double y, double z, double w) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = atan2(t0, t1);
    double t2 = +2.0 * (w * y - z * x);
    t2 = + t2 > +1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch_y = asin(t2);
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = atan2(t3, t4);

    euler[0] = roll_x;
    euler[1] = pitch_y;
    euler[2] = yaw_z;

    // Convert to degrees if needed
    // roll_x = roll_x * 180 / 3.14159
    // pitch_y = pitch_y * 180 / 3.14159
    // yaw_z = yaw_z * 180 / 3.14159
}

//void ApogeePrediction::calcVelocity(double acceleration) {
//    //previous curTime code was micros(), so I kept the cast as micros but can be nanos
//    double curTime = (double)(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count()); 
//    double deltaTime = (curTime - lastRecTime) / pow(10, 6);
//    currentVelocity += acceleration * deltaTime;
//    lastRecTime = curTime;
//}


//double ApogeePrediction::predictApogee(double* acceleration, double* orientation, double pressure, double temperature, double altitude) {
//    double euler[3] = {0, 0, 0};
//    eulerFromQuaternion(euler, orientation[0], orientation[1], orientation[2], orientation[3]);  // Convert to radians
//    double azvect = -acceleration[0] * sin(euler[1]) + acceleration[1]*cos(euler[1])*sin(euler[0]) + acceleration[2] * cos(euler[1])*cos(euler[0]); // vertical acceleration component, from the bno08x
//    calcVelocity(azvect);
//    
//    double rho = pressure*100/(287.058*(temperature+273.15)); // Get dry air density
//    double k = 0.5*rho*dragCoefficient*crossArea;
//    double predApogee = ((rocketMass/(2*k))*log((rocketMass*9.807 + k*pow(currentVelocity,2))/(rocketMass*9.807))+altitude); // Apogee prediction in meters
//
//    return predApogee;
//}

//mass of rocket - rocketMass (kg)
//drag co - dragCoefficient ()
//cross sectional area - crossArea (m^2)
//air density --> pressure (Pa), temperature (C)
// cV - m/s
//A - m
double ApogeePrediction::newPredictApogee(float currentVelocity, float altitude, float pressure, 
float temperature, float dragCoefficient, float rocketMass, float crossArea){
    //std::cout<<currentVelocity<<","<<altitude<<","<<pressure<<","<<temperature<<std::endl;
    //
    //DC: 0.68
    //2.132kg for 1
    //1.75kg for 2
    //CSA: 0.00535205381m^2
    //make these ^^ class values 
    //double dragCoefficient = 0.68;
    //double rocketMass = 1.75;
    //double crossArea = 0.00535205381;

    //green within 10
    //yellow within 50
    //red within 100

    // pressure param is pascal, deleted the *100 because the input should convert from mbar to Pa
    // temperature param is in celsius so we add 273.15 to convert to kelvin 
    double rho = pressure/(287.058*(temperature+273.15)); // Get dry air density
    double k = 0.5*rho*dragCoefficient*crossArea;
    double predApogee = ((rocketMass/(2*k))*log((rocketMass*9.807 + k*pow(currentVelocity,2))/(rocketMass*9.807))+altitude); // Apogee prediction in meters
    return predApogee;
}