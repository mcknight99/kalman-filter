#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

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
    double time;               // t
    double altitude;           // m
    double velocity;           // m/s
    double acceleration;       // m/s/s
    double temperature;        // C
    double pressure;           // mbar (mbar*100 = Pa needed for appred)
    double simulationTimeStep; // s

    friend std::ostream &operator<<(std::ostream &os, const Telemetry &telemetry)
    {
        os << telemetry.time << "," << telemetry.altitude << "," << telemetry.velocity << "," << telemetry.acceleration << "," << telemetry.temperature << "," << telemetry.pressure << "," << telemetry.simulationTimeStep;
        if (telemetry.isEvent)
        {
            os << telemetry.event;
        }
        return os;
    }
};

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
        std::cout << "Line: " << line << std::endl;
        if (line[0] == '#' && line[2] == 'E')
        {
            std::string firstWord = line.substr(8, line.find(' ', 8) - 8);
            // time double is found between "t=" and " seconds"
            std::string time_string = line.substr(line.find("t=") + 2, line.find(" seconds") - line.find("t=") - 2);
            double time = std::stod(time_string);

            file.rows.push_back({true, firstWord, time, -1, -1, -1, -1, -1});
        }
        else if (line[0] != '#')
        {
            // The way this else-if works is a bit sketchy, but it assumes that any non-comment line is a data line.
            // Could presumably crash with incorrect data
            std::string time, altitude, velocity, acceleration, temperature, pressure, simulationTimeStep;
            std::stringstream ss(line);
            std::getline(ss, time, ',');
            std::getline(ss, altitude, ',');
            std::getline(ss, velocity, ',');
            std::getline(ss, acceleration, ',');
            // mass is not used in the current implementation
            // drag coefficient is not used in the current implementation
            std::getline(ss, temperature, ',');
            std::getline(ss, pressure, ',');
            std::getline(ss, simulationTimeStep, ',');

            if (std::stod(altitude) > file.apogee)
            {
                file.apogee = std::stod(altitude);
            }

            file.rows.push_back({false, "", std::stod(time), std::stod(altitude), std::stod(velocity), std::stod(acceleration), std::stod(temperature), std::stod(pressure), std::stod(simulationTimeStep)});
        }
        else
        {
            std::cout << "unexpected line: " << line << std::endl;
            // the last thing line could be is the very first line which should be the field descriptions if the data was exported correctly
            if (line != "# Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s²),Air temperature (°C),Air pressure (mbar),Simulation time step (s)")
            {
                // But if it isn't the exact field descriptions, his is either an unwanted line in the data or the data is incorrect
                // throw error is optional, but it is useful to know if the file is not formatted correctly
                std::cerr << "Error: Unexpected line in file or incorrect fields: " << line << std::endl;
            }
        }
    }
}

int main()
{
    std::cout << "Copy and paste your CSV file path here:";
    std::string csvFile;
    std::cin >> csvFile;

    std::cout << "Reading file: " << csvFile << std::endl;

    File<Telemetry> inputFile;
    inputFile.name = csvFile;
    inputFile.file.open(inputFile.name);
    if (!inputFile.file)
    {
        std::cerr << "Failed to open the input CSV file." << std::endl;
        return 1;
    }
    readFile(inputFile);

    double apogee = inputFile.apogee;

    inputFile.file.close();

    std::ofstream outputFile;
    outputFile.open(csvFile + "_with_apogee.csv", std::ios::out | std::ios::trunc);
    if (!outputFile)
    {
        std::cerr << "Failed to open the output CSV file." << std::endl;
        return 1;
    }
    // Write the first line
    outputFile << "# Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s²),Air temperature (°C),Air pressure (mbar),Simulation time step (s),Apogee (m)" << std::endl;

    // Write the data to the output file
    for (int i = 0; i < inputFile.rows.size(); i++)
    {
        if(inputFile.rows[i].isEvent) {
            outputFile << "# Event " << inputFile.rows[i].event << " occurred at t=" << inputFile.rows[i].time << " seconds" << std::endl;
        } else {
            outputFile << inputFile.rows[i] << "," << apogee << std::endl;
        }
    }

    std::cout << "Apogee column added successfully." << std::endl;

    return 0;
}