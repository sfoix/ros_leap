
#include "leap_gestures/GestureInterpreter.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leap_gesture_interpreter");
    ros::NodeHandle nh;

    leap::GestureInterpreter gi(nh);
    gi.run();

    return 0;
}
