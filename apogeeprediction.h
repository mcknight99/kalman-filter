#ifndef APOGEEPRED_H
#define APOGEEPRED_H

class ApogeePrediction {
  private:
    double rocketMass;
    double dragCoefficient;
    double crossArea; // In m^2
    double targetApogee;

    double predApogee;
    double currentVelocity;
    double lastRecTime;
    
    void eulerFromQuaternion(double *euler, double x, double y, double z, double w);
    //void calcVelocity(double acceleration);
  public:
    //ApogeePrediction(double rocketMass, double dragCoefficient, double crossArea, double targetApogee);
    //double predictApogee(double* acceleration, double* orientation, double pressure, double temperature, double altitude);
    static double newPredictApogee(float currentVelocity, float altitude, float pressure, 
float temperature, float dragCoefficient, float rocketMass, float crossArea);
};

#endif