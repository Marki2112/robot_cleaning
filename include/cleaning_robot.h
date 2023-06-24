#ifndef KAERCHER_H_
#define KAERCHER_H_

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <string>
#include <cmath>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/console.h>

struct Point {
    double x;
    double y;
};

class cleaning_robot{

    public:
        cleaning_robot();
        ~cleaning_robot();

        double calculateCleaningArea(Json::Value lengthCleaningGadget, std::vector<double> distance);
        double calculateTimeOfPath(std::vector<double> distance);
        double calculatePath(std::vector<double> distance);
        std::vector<double> calculateDistance(Json::Value robot_path);
        Json::Value readJsonFile(std::string jsonfile);
        
    private:

        double calculateVelocity(double Kappa);
        std::vector<Point> createVector(Json::Value robot_path);
        double calculateLength(const Point& start, const Point& end);
};

cleaning_robot::cleaning_robot(){

}

cleaning_robot::~cleaning_robot(){  

}


#endif