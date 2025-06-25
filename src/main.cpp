#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include "controller.hpp"

#define DEFAULT_KP 5.0 // Proportional gain
#define DEFAULT_KI 0.5 // Integral gain
#define DEFAULT_KD 1.0 // Derivative gain

#define TEMPERATURE 5.0
#define TARGET_TEMPERATURE 30.0
#define DT 1.0 // Delta time in seconds
#define STEPS 50
#define MAX_HEATER_POWER 100.0

#define HEATING_COEFF 0.1
#define LOSS_COEFF 0.02

double updateRealTemperature(double currentTemp, double heaterPercent, double dt, double ambientTemp)
{
    double heatInput = HEATING_COEFF * heaterPercent;
    double heatLoss = LOSS_COEFF * (currentTemp - ambientTemp);

    return currentTemp + (heatInput - heatLoss) * dt;
}

int main()
{
    std::cout << "Starting the controller application." << std::endl;

    std::string mode;
    std::cout << "Enter controller mode [P / PI / PID]: ";
    std::cin >> mode;

    std::unique_ptr<Controller> controller;
    if (mode == "P" || mode == "p")
    {
        controller = std::make_unique<PController>(DEFAULT_KP);
    }
    else if (mode == "PI" || mode == "pi")
    {
        controller = std::make_unique<PIController>(DEFAULT_KP, DEFAULT_KI);
    }
    else if (mode == "PID" || mode == "pid")
    {
        controller = std::make_unique<PIDController>(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }
    else
    {
        std::cout << "Invalid mode selected. Exiting." << std::endl;
        return 1;
    }

    std::ofstream outputFile("log.csv");
    if (!outputFile.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        return 1;
    }

    outputFile << "Time, Temperature\n";

    double ambientTemp = TEMPERATURE;
    double currentTemp = ambientTemp;

    for (int i = 0; i < STEPS; ++i)
    {
        double error = TARGET_TEMPERATURE - currentTemp;
        double controllerOutput = controller->update(error, DT);

        double heaterPercent = controllerOutput;
        heaterPercent = std::max(0.0, std::min(heaterPercent, MAX_HEATER_POWER));

        currentTemp = updateRealTemperature(currentTemp, heaterPercent, DT, ambientTemp);

        double time = i * DT;
        std::cout << "Time: " << time << "s, Temperature: " << currentTemp << "C, Heater: " << heaterPercent << "%" << std::endl;
        outputFile << time << ", " << currentTemp << "\n";
    }

    outputFile.close();
    return 0;
}