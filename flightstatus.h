#ifndef FLIGHTSTATUS_H
#define FLIGHTSTATUS_H

#include <deque>
#include <vector>
#include <algorithm>
#include <string>

enum Stage {
    ARMED,
    ASCENT,
    COAST,
    APOGEE,
    DESCENT,
    ONGROUND,
};

class FlightStatus {
  private:
    Stage flightStage;
    std::deque<double> altitudeDeque;

    double median(std::vector<double> vec);
    bool checkApogee();
    bool checkGround();
  public:
    FlightStatus();
    void newTelemetry(double acceleration, double altitude);
    Stage getStage();
    std::string getStageString();
};

#endif