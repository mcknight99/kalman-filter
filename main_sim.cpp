// This code is designed to use OpenRocket simulations
// to determine the robustness and accuracy of the code

// Testing: find upper and lower bounds for different simulation variables (mass, drag coefficient, cross-sectional area, apogee) or weather variables (pressure, temperature, wind speed, etc)

// Current goals:
// Flight Status detection
// Velocity Calculations (EKF)
// Apogee Prediction accuracy
// Time into coast phase for accurate apogee prediction
// AHRS

// Known bugs:
// OpenRocket does not simulate uT/magnetometer data (or at least doesn't export or simulate by default, there may be a plugin)
//      This means that the AHRS cannot be tested on this simulated data

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "flightstatus.h"
#include "kf-2d.h"
#include "apogeeprediction.h"
#include "ahrs.h"

#include "windows.h" //Sleep(ms) for real sim time

// Color modifiers for terminal
struct colors
{
    const std::string DEFAULT_COLOR = "\033[0m";       // white text
    const std::string YELLOW_BOLD = "\033[1;33m";      // yellow text
    const std::string TEAL = "\033[1;36m";             // teal text
    const std::string BACKGROUND_COLOR = "\u001B[40m"; // black highlight background
    const std::string RED = "\x1B[31m";
    const std::string GREEN = "\x1B[32m";
    const std::string YELLOW = "\x1B[33m";
    const std::string BLUE = "\x1B[34m";
    const std::string MAGENTA = "\x1B[35m";
    const std::string CYAN = "\x1B[36m";
    const std::string WHITE = "\x1B[37m";
    const std::string GREEN_BACKGROUND = "\033[3;42;30m";
    const std::string YELLOW_BACKGROUND = "\033[3;43;30m";
    const std::string BLUE_BACKGROUND = "\033[3;44;30m";
    const std::string CYAN_BACKGROUND = "\033[3;104;30m";
    const std::string GRAY_BACKGROUND = "\033[3;100;30m";
    const std::string RED_BACKGROUND = "\033[3;101;30m";
};

// Struct to store telemetry data
// event string is used when a # is found at the start of the line. when there is no #, nothing is stored in the event string
//      but if there is a #, the full caps after the word "Event" is stored in the event string
// time is the time elapsed in seconds
// altitude is the altitude in meters
// velocity is the velocity in m/s
// acceleration is the acceleration in m/s^2
// temperature is the temperature in Celsius
// pressure is the pressure in mbar
struct Telemetry
{
    bool isEvent = false;
    std::string event;
    double time;         // t
    double altitude;     // m
    double velocity;     // m/s
    double acceleration; // m/s/s
    double mass;         // g (g/1000 = kg needed for appred)
    double dragCoefficient;
    double temperature; // C
    double pressure;    // mbar (mbar*100 = Pa needed for appred)
};
double crossSectionalArea = 0.0198556341975; // m^2

// @param name: name of the file
// @param file: file object to read from
// @param rows: vector of telemetry data
// @param apogee: apogee of the rocket\n
//
// using templates to allow for different types of telemetry data
template <typename T>
struct File
{
    std::string name;
    std::ifstream file;
    std::vector<T> rows;
    double apogee = -1;
};

// @param file: file object to read from
// Read the data from the file and store it in the data vector
// Lines starting with a # are comments and can be used evaluate the simulation. The rest of the telemetry is -1
// Otherwise, the data is stored in the telemetry struct and the event string is nothing
void readFile(File<Telemetry> &file)
{
    std::cout << "Reading File: " << file.name << std::endl;
    std::string line;
    while (std::getline(file.file, line))
    {
        if (line[0] == '#' && line[2] == 'E')
        {
            std::string firstWord = line.substr(8, line.find(' ', 8) - 8);
            // time double is found between "t=" and " seconds"
            std::string time_string = line.substr(line.find("t=") + 2, line.find(" seconds") - line.find("t=") - 2);
            double time = std::stod(time_string);

            file.rows.push_back({true, firstWord, time, -1, -1, -1, -1, -1, -1, -1});
        }
        else if (line[0] != '#')
        {
            // The way this else-if works is a bit sketchy, but it assumes that any non-comment line is a data line.
            // Could presumably crash with incorrect data
            std::string time, altitude, velocity, acceleration, mass, dragCoefficient, temperature, pressure;
            std::stringstream ss(line);
            std::getline(ss, time, ',');
            std::getline(ss, altitude, ',');
            std::getline(ss, velocity, ',');
            std::getline(ss, acceleration, ',');
            std::getline(ss, mass, ',');
            std::getline(ss, dragCoefficient, ',');
            std::getline(ss, temperature, ',');
            std::getline(ss, pressure, ',');

            if (std::stod(altitude) > file.apogee)
            {
                file.apogee = std::stod(altitude);
            }

            file.rows.push_back({false, "", std::stod(time), std::stod(altitude), std::stod(velocity), std::stod(acceleration), std::stod(mass), std::stod(dragCoefficient), std::stod(temperature), std::stod(pressure)});
        }
        else
        {
            std::cout << "unexpected line: " << line << std::endl;
            // the last thing line could be is the very first line which should be the field descriptions if the data was exported correctly
            if (line != "# Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s²),Mass (g),Drag coefficient (​),Air temperature (°C),Air pressure (mbar)")
            {
                // But if it isn't the exact field descriptions, his is either an unwanted line in the data or the data is incorrect
                // throw error is optional, but it is useful to know if the file is not formatted correctly
                std::cerr << "Error: Unexpected line in file or incorrect fields: " << line << std::endl;
            }
        }
    }
}

double percentError(double prediction, double actual)
{
    return abs(100 * ((prediction - actual) / actual));
}

// @param prediction: predicted value
// @param actual: actual value
// @return: color code for the terminal
//     magenta: within 1%
//     green: within 2%
//     yellow: within 3%
//     red: within 4%
std::string colorGradePrediction(double prediction, double actual)
{
    colors colors;
    // percentages double values must be sorted by ascending value
    std::vector<std::pair<double, std::string>> percentages = {
        {1, colors.MAGENTA},
        {2, colors.GREEN},
        {3, colors.YELLOW},
        {4, colors.RED}};

    double error = percentError(prediction, actual);
    for (auto percent : percentages)
    {
        if (percent.first > error && error >= 0)
        {
            return percent.second;
        }
    }
    return colors.DEFAULT_COLOR;
}

void simulateFile(File<Telemetry> &file)
{
    colors colors;
    std::cout << "Simulating File: " << file.name << std::endl;
    std::cout << "\tApogee: " << file.apogee << std::endl;
    // File sim needs to be setup before the loop
    FlightStatus fs;
    AHRS ahrs;
    KF2D kf;

    std::ofstream outputFile = std::ofstream("ORK simulated data/" + file.name + "_output.csv");

    if(!outputFile.is_open())
    {
        std::cerr << "Failed to open output file" << std::endl;
        return;
    }

    // output file format: time, altitude, vertical velocity, vertical acceleration, air temperature, air pressure, simulation time step, predicted apogee
    outputFile << "# Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s²),Air temperature (°C),Air pressure (mbar),Simulation time step (s),Apogee (m)\n";
    

    float last_time = 0;
    bool initialized = false;
    for (size_t i = 0; i < file.rows.size(); i++)
    {

        Telemetry &data = file.rows[i];
        if (data.isEvent)
        {
            std::cout << colors.YELLOW_BOLD << "Time: " << data.time << " Event: " << data.event << colors.DEFAULT_COLOR << std::endl;
            // Events are listed before the data line, so we can skip the line for entering data and see how accurate our prediction is to the event's time
            // Ideally, the prediction would happen at the same time as the event, but that is nigh impossible with real world problems

            if (data.event == "RECOVERY_DEVICE_DEPLOYMENT")
            {
                std::cout << "Skipping descent\n";
                break; // skip the descent
            }

            outputFile << "# Event " << data.event << " occurred at t=" << data.time << " seconds\n";
        }
        else
        {
            float this_time = data.time;
            float delta_time = this_time - last_time;
            KF2D::MeasurementVector measurement = {(float)data.altitude, (float)data.acceleration};
            if (!initialized)
            {

                kf.InitializeKalmanFilter(measurement);
                initialized = true;
            }
            else
            {
                kf.Update(measurement, delta_time);
            }
            KF2D::StateVector prediction = kf.getPrediction();
            std::cout << "Time: " << data.time << "s, " << data.altitude << "m, " << data.velocity << "m/s, " << data.acceleration << "m/s/s, " << data.temperature << "*C,  " << data.pressure << "mbar -> ";

            fs.newTelemetry(data.acceleration, data.altitude);

            double appred = ApogeePrediction::newPredictApogee(prediction.y, prediction.x, data.pressure * 100, data.temperature, data.dragCoefficient, data.mass / 1000, crossSectionalArea);

            if (fs.getStageString() == "COAST")
            {
                std::cout << colors.GREEN_BACKGROUND;
            }
            std::cout << fs.getStageString() << colors.DEFAULT_COLOR << " ";
            std::cout << "KF: " << colorGradePrediction(prediction.x, data.altitude) << prediction.x << "m, "
                      << colorGradePrediction(prediction.y, data.velocity) << prediction.y << "m/s, "
                      << colorGradePrediction(prediction.z, data.acceleration) << prediction.z << "m/s/s ";
            if (appred > file.apogee + 100)
            {
                std::cout << colors.RED_BACKGROUND;
            }
            else
            {
                std::cout << colorGradePrediction(appred, file.apogee);
            }
            std::cout << "appred=" << appred << colors.DEFAULT_COLOR;
            Sleep(40);
            std::cout << std::endl;

            //write to the output file with estimated state of the rocket (altitude, velocity, acceleration) and the predicted apogee
            outputFile<< data.time << "," << prediction.x << "," << prediction.y << "," << prediction.z << "," << data.temperature << "," << data.pressure << "," << delta_time << "," << appred << "\n";
            
            last_time = this_time;
        }
    }
    outputFile.close();

}

// @param files: vector of files to simulate
// Run through each file and read the data.
// Run the simulation using the data
// Forget the data at the end to save memory
void simulateFileList(std::vector<File<Telemetry>> &files)
{
    for (size_t i = 0; i < files.size(); i++)
    {
        readFile(files[i]);
        simulateFile(files[i]);
        files[i].rows.clear();
    }
}

int main()
{
    // OpenRocket Simulation Data files
    std::vector<File<Telemetry>> files;
    // files.push_back({"sim1", std::ifstream("simulated data/ORK sim example.csv"), std::vector<Telemetry>()});
    // files.push_back({"sim2", std::ifstream("simulated data/ORK real example.csv"), std::vector<Telemetry>()});
    files.push_back({"sim3", std::ifstream("ORK simulated data/SAC NM test sim with extra vars.csv"), std::vector<Telemetry>()});

    simulateFileList(files);
}