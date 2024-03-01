#include "flightstatus.h"

FlightStatus::FlightStatus(): altitudeDeque(128, 0) {
    flightStage = ARMED;
}

double FlightStatus::median(std::vector<double> vec){
    int size = vec.size();
    sort(vec.begin(), vec.end());
    if (size % 2 != 0)
        return (double)vec[size/2];
    return (double)(vec[(size-1)/2] + vec[size/2])/2.0;
}


bool FlightStatus::checkApogee() {
    std::vector<double> lm(altitudeDeque.cend() - 16, altitudeDeque.cend());
    std::vector<double> fm(altitudeDeque.cend() - 48, altitudeDeque.cend() - 16);

    double lmMed = median(lm);
    double fmMed = median(fm);

    return lmMed < fmMed;
}

bool FlightStatus::checkGround() {
    std::vector<double> lm(altitudeDeque.cend() - 16, altitudeDeque.cend());

    double lmMed = median(lm);

    return lmMed < 20;
}

void FlightStatus::newTelemetry(double acceleration, double altitude) {
    altitudeDeque.pop_front();
    altitudeDeque.push_back(altitude);

    if(acceleration > 11 && flightStage == ARMED) {
        flightStage = ASCENT;
    }
    if(acceleration < 0 && flightStage == ASCENT) {
        flightStage = COAST;
    }
    if(checkApogee() && flightStage == COAST)
    {
        flightStage = APOGEE;
    }
    if(flightStage == APOGEE) {
        flightStage = DESCENT;
    }
    if(checkGround() && flightStage == DESCENT) {
        flightStage = ONGROUND;
    }
}

Stage FlightStatus::getStage() {
    return flightStage;
}


