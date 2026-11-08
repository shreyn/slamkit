#pragma once //header guards

struct Pose2D {
    double x;
    double y;
    double theta;
};

Pose2D compose(Pose2D a, Pose2D b); //returns in map frame

double wrap_angle(double angle);

Pose2D inverse(Pose2D p); 

//where is pose b, as seen from pose a?
Pose2D relative_pose(Pose2D a, Pose2D b);