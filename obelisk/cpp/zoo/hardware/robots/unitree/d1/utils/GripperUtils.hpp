#pragma once
#include <cmath>

// length (meters) of the servo blade
const double A = 0.02;
// length (meters) of the link between the servo blade and the screw attached to the gripper clamp
const double C = 0.03;
// horizontal displacement (meters) between the screw and the inner the face of the clamp
const double OFFSET = -0.0125; 

double degrees2meters(double servo_angle) {
    /*
    Converts the gripper servo angle `servo_angle` from degrees to the gripper
    position in meters. The gripper position is the half of the distance between
    the two clamps of the gripper.
    */
    double theta = servo_angle * M_PI / 180;
    // horizontal displacement (meters) between the screw and the center of
    // the gripper
    double B = A * std::sin(theta) + std::sqrt(std::pow(A * std::sin(theta), 2) + C * C - A * A);
    double pos = B + OFFSET;
    return pos;
}

double meters2degrees(double pos) {
    /*
    Converts the gripper position in meters to the gripper servo angle `servo_angle` 
    in degrees. The gripper position is the half of the distance between
    the two clamps of the gripper.
    */
    double B = pos - OFFSET;
    double D = (A * A + B * B - C * C) / (2 * A * B);
    double theta = std::asin(D);
    double servo_angle = theta * 180 / M_PI;
    return servo_angle;
}