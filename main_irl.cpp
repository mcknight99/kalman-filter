#include "apogeeprediction.h"
#include "ahrs.h"
#include "windows.h"
#include "kf-2d.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>

#include <thread>
#include <chrono>

AHRS ahrs;
AHRSMap ahrsIn;   // will be in the format of gyro xyz, accel xyz, mag xyz
AHRSMap ahrsData; // will be in the format of
KF2D KF = KF2D();

bool printAHRS = false;
bool printKF = true;

KF2D::MeasurementVector measurement;
std::vector<std::string> files;
std::vector<float> recordedApogees;    // max altitude from each file in m
std::vector<float> recordedMasses;     // rocket mass from each file in kg
std::vector<float> recordedCrossAreas; // rocket cross areas from each file in m2
std::vector<float> recordedDragCoefficients;
float timeElapsed = 0;
float earliestGreenTime = 0;
int earliestGreenIndex = 0;

float lastV = 0;
float lastAKF = 0;
float lastA = 0;

// prototypes
void setup();
void loop(const KF2D::MeasurementVector &measurement, int fileNum);

// its better to have a yellow estimate over the appred and desired apogee
// no its better to underestimate so we dont activate brakes when the real apogee is lower than we think

int main()
{
  setup();
  // loop while there is more csv to read
  // do a for each csv? list of csvs and iterate thru each? --> end game goal --> func for iterating one sheet (loop() for a loaded csv until no more)
  for (size_t i = 0; i < files.size(); i++)
  { // starting code for looping through files
    KF = KF2D();
    measurement = {0,0};
    KF.InitializeKalmanFilter(measurement);
    timeElapsed = 0;
    earliestGreenTime = 0;
    earliestGreenIndex = 0;
    ahrsIn.clear();
    ahrsData.clear();

    ahrs.begin(333333333333.0F); // sample frequency 
    ahrs.setRotationVector(0, 0, 0);

    lastV = 0;
    lastAKF = 0;
    lastA = 0;
    std::ifstream inputFile(files[i]);
    if (inputFile.is_open())
    {
      std::cout << "\x1B[36mOpened file " << files[i] << "\n################################################################################\033[0m\n";
      std::string line;
      int rowNum = 0; // should ideally be within 15 rows less than 3m, 10-15 is standard from sims
      float last_time = 0;
      float apogee = 0;
      while (std::getline(inputFile, line)) // start code for looping through data in files
      {

        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> row;
        int colNum = 0;
        float this_time;    // grab the time from the csv
        float dt = -1000000; // just so if something goes wrong it goes horribly wrong
        // measurement = {col11, col123 ? better accel would be from AHRS };
        //  better accel (case 1, [1]) would be absolute from AHRS but that functionality isnt in AHRS yet(?)

        float pressure;
        float temperature;

        while (std::getline(ss, field, ','))
        { // grab row
          if (rowNum > 0)
          { // skip dead rows (text)
            float field_f = std::stof(field);
            switch (colNum)
            { // column data pushed to respective variables
            // this switch is grody as heck but im not sure of a more efficient way except for another hash map but tbh this is more readable and fixable
            case 0:                                     // time
              this_time = std::atof(data(field)) / 1E9; // dividing by 1E9 converts from ns to s (dt is in ns) - for some reason nums work at 1e12 check math
              dt = this_time - last_time;
              if (rowNum > 1)
              {
                timeElapsed += dt;
              }
              break;
            case 10: // altitude
              measurement[0] = field_f;
              if (measurement[0] > apogee)
              {
                apogee = measurement[0];
              }
              break;
            case 1: // acceleration (x)
              measurement[1] = field_f - 9.81;
              // ahrsIn["accelx"] = measurement[1];
              ahrsIn["accelx"] = field_f; // this gives all near-zeroes so i assume it needs one without g
              break;
            case 2: // accely
              ahrsIn["accely"] = field_f;
              break;
            case 3: // accelz
              ahrsIn["accelz"] = field_f;
              break;
            case 4:
              ahrsIn["gyrox"] = field_f;
              break;
            case 5:
              ahrsIn["gyroy"] = field_f;
              break;
            case 6:
              ahrsIn["gyroz"] = field_f;
              break;
            case 7:
              ahrsIn["magx"] = field_f;
              break;
            case 8:
              ahrsIn["magy"] = field_f;
              break;
            case 9:
              ahrsIn["magz"] = field_f;
              break;
            case 11: // pressure
              pressure = field_f;
              break;
            case 12: // temp
              temperature = field_f;
              break;
            default: // if it isnt being recorded, push it to the row storage to be printed
              std::cout << "default??" << field;
              row.push_back(std::to_string(colNum));
              row.push_back(":");
              row.push_back(field); // field is the measurement value that can be turned to float
              row.push_back(", ");
              break;
            } // close data switch
          }   // skip rows closer
          colNum++;
        } // close row grabber

        if (rowNum == 1) // skipping row 0 which is column names in the csv. real use will be initializing on the first readings and updating after
        {

          std::cout << "\n\n\n\n\n"; 
          std::cout<<measurement[0] << ", " << measurement[1] << ", " << KF.getPrediction()[0] << ", " << KF.getPrediction()[1] << ", " << KF.getPrediction()[2]<<std::endl;
          KF.InitializeKalmanFilter(measurement); // there may be a small chance that back to back KF will "smudge" if it doesnt fully clean from this line
            std::cout<<"FILTER RESET ";
          std::cout<<measurement[0] << ", " << measurement[1] << ", " << KF.getPrediction()[0] << ", " << KF.getPrediction()[1] << ", " << KF.getPrediction()[2]<<std::endl;
          std::cout<<"\n\n\n\n\n";
        }
        else if (rowNum > 1)
        {
          Sleep(dt*1000);
          loop(measurement, i);
        }
        else
        {
          // if rowNum==1, ignore it to initialize dt. cant have a dt if there was no ti
        }

        // PREDICT APOGEE HERE
        // std::cout<<"pt:"<<pressure<<","<<temperature<<std::endl;
        // replacing measurement[0] with KF.x_hat[0]
        double predApogee = ApogeePrediction::newPredictApogee(KF.getPrediction()[1], KF.getPrediction()[0], pressure, temperature, recordedDragCoefficients[i], recordedMasses[i], recordedCrossAreas[i]);

        // 2.5676298406m tall
        if (recordedApogees[i] - 2.568 <= predApogee && predApogee <= recordedApogees[i] + 2.568)
        {
          //\x1B[35m
          std::cout << "\t\x1B[35mappred:" << predApogee << "m\033[0m\n"; // magenta within rocket height
        }
        else if (recordedApogees[i] - 10 <= predApogee && predApogee <= recordedApogees[i] + 10)
        {
          std::cout << "\t\x1B[32mappred:" << predApogee << "m\033[0m\n"; // green within 10
          if (earliestGreenIndex == 0)
          {
            earliestGreenIndex = rowNum;
            earliestGreenTime = timeElapsed;
          }
        }
        else if (recordedApogees[i] - 50 <= predApogee && predApogee <= recordedApogees[i] + 50)
        {
          std::cout << "\t\x1B[33mappred:" << predApogee << "m\033[0m\n"; // yellow within 50
        }
        else if (recordedApogees[i] - 100 <= predApogee && predApogee <= recordedApogees[i] + 100)
        {
          std::cout << "\t\x1B[31mappred:" << predApogee << "m\033[0m\n"; // red within 100
        }
        else
        {
          std::cout << "\tappred:" << predApogee << "m\n"; // white outside 100
        }

        rowNum++;
        last_time = this_time; // im just a moron who switched these up for hours and didnt know why the measurements were buggin
      }
      std::cout << "\x1B[32m" << files[i] << " apogee is " << apogee << "m\033[0m\n";
      std::cout << "\x1B[32m Earliest Green Index (+-10m): Row " << earliestGreenIndex << ", after " << earliestGreenTime << "s\033[0m\n";
    }
    else
    { // if the file wasn't opened
      std::cerr << "\x1B[36mError opening file " << files[i] << "\033[0m\n";
      // return 1;
    } // inputFile is open if/else closer
  }

  // printf("\x1B[31mTesting\033[0m\t\t"); //print red text
  bool test = true;
  std::cout << !test << std::endl;
  std::cout << !!test << std::endl;
}

void setup()
{

  // Would be cool to make these into #defines as custom escape codes
  // like this
  // #define REDTEXT \x1B[31m;
  // #define CLEARFORMAT \033[0m;
  /*
  printf("\n");
  printf("\x1B[31mRed\033[0m\t\t");
  printf("\x1B[32mGreen\033[0m\t\t");
  printf("\x1B[33mYellow\033[0m\t\t");
  printf("\x1B[34mDark Blue\033[0m\t\t");
  printf("\x1B[35mMagenta\033[0m\n");

  printf("\x1B[36mCyan\033[0m\t\t");
  printf("\x1B[36mTexting\033[0m\t\t");
  printf("\x1B[36mTexting\033[0m\t\t");
  printf("\x1B[37mWhite\033[0m\t\t");
  printf("\x1B[93mTexting\033[0m\n");

  printf("\033[3;42;30mTexting\033[0m\t\t");
  printf("\033[3;43;30mTexting\033[0m\t\t");
  printf("\033[3;44;30mTexting\033[0m\t\t");
  printf("\033[3;104;30mTexting\033[0m\t\t");
  printf("\033[3;100;30mTexting\033[0m\n");

  printf("\033[3;47;35mTexting\033[0m\t\t");
  printf("\033[2;47;35mTexting\033[0m\t\t");
  printf("\033[1;47;35mTexting\033[0m\t\t");
  printf("\t\t");
  printf("\n");
  */

  // put your setup code here, to run once:

  std::cout << "\x1B[36mStarting up\n";
  files = {"custom_uno.csv", "custom_dos.csv"};
  // files = {"custom_dos.csv"};
  recordedApogees = {577.288, 331.847};
  recordedMasses = {1.75, 2.132};
  recordedCrossAreas = {0.00535205381, 0.00535205381};
  recordedDragCoefficients = {0.68, 0.68};

  // sdLogger.setup();

  // replace the measurement with fstream data (here and in loop)
  // the next 2 lines should be run when launch state is detected instead of at start
  // measurement = {telemData.sensorData["altitude"].altitude, (-1)*telemData.sensorData["acceleration"].acceleration.z}; //want y then ay

  std::cout << "Finished setup\033[0m\n";
}

void loop(const KF2D::MeasurementVector &measurement, int fileNum)
{
  //..telemData = telemetry.getTelemetry(); //change to get next data from fstream

  ahrs.update(ahrsIn["gyrox"], ahrsIn["gyroy"], ahrsIn["gyroz"],
              ahrsIn["accelx"], ahrsIn["accely"], ahrsIn["accelz"],
              ahrsIn["magx"], ahrsIn["magy"], ahrsIn["magz"]);

  float rx, ry, rz;
  ahrs.getRotationVector(&rx, &ry, &rz);
  // Serial.printf("rx=%f \try=%f \trz=%f", rx, ry, rz);
  float gx, gy, gz;
  ahrs.getGravityVector(&gx, &gy, &gz);
  // Serial.printf("\tgx=%f \tgy=%f \tgz=%f", gx, gy, gz);
  float ax, ay, az;
  ahrs.getLinearAcceleration(&ax, &ay, &az);

  if (printAHRS)
  {
    if (measurement[0] == recordedApogees[fileNum])
    {
      std::cout << "\033[3;42;30mrx=" << rx << "\try=" << ry << "\trz=" << rz;
      std::cout << "\tgx=" << gx << "\tgy=" << gy << "\tgz=" << gz;
      std::cout << "\tax=" << ax * 9.81 << "\tay=" << ay * 9.81 << "\taz=" << az * 9.81 << "\033[0m\n";
    }
    else
    {
      std::cout << "rx=" << rx << "\try=" << ry << "\trz=" << rz;
      std::cout << "\tgx=" << gx << "\tgy=" << gy << "\tgz=" << gz;
      std::cout << "\tax=" << ax << "\tay=" << ay << "\taz=" << az << std::endl;
    }
  }

  ahrsData["rx"] = rx;
  ahrsData["ry"] = ry;
  ahrsData["rz"] = rz;
  ahrsData["gx"] = gx;
  ahrsData["gy"] = gy;
  ahrsData["gz"] = gz;
  ahrsData["ax"] = ax; // whichever a is the vertical needs to be put into the KF eventually
  ahrsData["ay"] = ay;
  ahrsData["az"] = az;
  // std::this_thread::sleep_for(250ms);

  // KF2D::MeasurementVector localMeasurement = {measurement[0], ahrsData["gx"]*9.81};

  KF.Update(measurement);
  KF.Predict();
  if (printKF)
  {
    if (measurement[0] == recordedApogees[fileNum])
    {
      std::cout << "\033[3;42;30mt= " << timeElapsed << "\t";
      std::cout << "altitude = " << measurement[0] << "\t y-acceleration = " << measurement[1] << "\t";
      std::cout << "x_hat: \t" << KF.getPrediction()[0] << " m, \t\x1B[31m" << KF.getPrediction()[1] << " m/s\033[3;42;30m, \t" << KF.getPrediction()[2] << " m/s/s\033[0m";
    }
    else
    {
      if (measurement[1] <= 0 && lastA >= 0 && lastA > 1) // motor burnout has yellow highlight
      {
        std::cout << "\033[3;43;30m";
      }
      std::cout << "t= " << timeElapsed << "\t";
      std::cout << "altitude = " << measurement[0] << "\t y-acceleration = " << measurement[1] << "\t";

      if (KF.getPrediction()[1] > lastV)
      { // if V is going up, green text
        std::cout << "x_hat: \t" << KF.getPrediction()[0] << " m, \t\x1B[32m" << KF.getPrediction()[1] << " m/s\033[0m, \t" << KF.getPrediction()[2] << " m/s/s";
      }
      else
      { // if V is going down, red text
        std::cout << "x_hat: \t" << KF.getPrediction()[0] << " m, \t\x1B[31m" << KF.getPrediction()[1] << " m/s\033[0m, \t" << KF.getPrediction()[2] << " m/s/s";
      }
    }

    lastV = KF.getPrediction()[1];
    lastAKF = KF.getPrediction()[2];
    lastA = measurement[1];
  }

  // std::this_thread::sleep_for(250ms);
}