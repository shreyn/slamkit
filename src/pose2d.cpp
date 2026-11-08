#include "slamkit/pose2d.hpp"
#include <cmath> //C: <math.h>


Pose2D compose(Pose2D a, Pose2D b) { 
    //a: robot's pose. b: motion in terms of robot's own view
    //have to rotate the motion b by a's direction, before adding to a's position
    Pose2D result;
    result.x = a.x + std::cos(a.theta)*b.x - std::sin(a.theta)*b.y;
    result.y = a.y + std::sin(a.theta)*b.x + std::cos(a.theta)*b.y;
    result.theta = wrap_angle(a.theta + b.theta);
    return result;
}

double wrap_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0*M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0*M_PI;
    }
    return angle;
}

Pose2D inverse(Pose2D p) {
    Pose2D result;
    result.x = -(std::cos(p.theta) * p.x + std::sin(p.theta) * p.y);
    result.y = std::sin(p.theta) * p.x - std::cos(p.theta) * p.y;
    result.theta = wrap_angle(-p.theta);
    return result;
}

Pose2D relative_pose(Pose2D a, Pose2D b) {
    return compose(inverse(a), b); //undo a, then apply b.
}