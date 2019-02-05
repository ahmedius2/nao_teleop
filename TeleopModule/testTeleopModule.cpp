#include <iostream>
#include <vector>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include <alproxies/albasicawarenessproxy.h>


#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include <unistd.h>
#include <cmath>

using namespace AL::Math;



int main(int argc, char **argv)
{
    std::string robotIp = "192.168.43.204";

    if (argc < 2) {
        std::cerr << "Usage: almotion_getangles robotIp "
                  << "(optional default \"169.254.67.213\")."<< std::endl;
    }
    else {
        robotIp = argv[1];
    }

    AL::ALMotionProxy motionProxy(robotIp);
    AL::ALRobotPostureProxy robotPostureProxy(robotIp);
    AL::ALBasicAwarenessProxy awarenessProxy(robotIp);

    motionProxy.setBreathEnabled("Body", false);
    motionProxy.setIdlePostureEnabled("Body", false);
    awarenessProxy.stopAwareness();
    motionProxy.setSmartStiffnessEnabled(true);
    motionProxy.setMoveArmsEnabled(false, false);

    robotPostureProxy.goToPosture("StandInit",0.5f);
//    motionProxy.stiffnessInterpolation(AL::ALValue::array("LArm","RArm"),
//                                       0.5f, 0.75f);

    // Arm Angles for cart teleoperation
    //const float predefArmAngles[12] = { // LArm and RArm
    //    0.391128, -0.135034, -1.46348, -0.831386, 1.58305, 0.622,
    //    0.34826, 0.124212, 1.50174, 0.750168, -1.53251, 0.664};
//    const float predefArmAngles[12] = { // LArm and RArm
//        -0.375,  0.1, 0, -0.5, 0, 0, // 0,  0.5, 0, -0.75, 0, 0.6, default
//        -0.375, -0.1, 0,  0.5, 0, 0};// 0, -0.5, 0,  0.75, 0, 0.6  values
//    std::vector<float> armAnglesVec;
//    armAnglesVec.assign(predefArmAngles,
//                           predefArmAngles+12);

//    motionProxy.angleInterpolationWithSpeed(AL::ALValue::array("LArm","RArm"),
//                                            armAnglesVec,0.4f);

    motionProxy.post.openHand("RHand"); motionProxy.openHand("LHand");
    char ch; std::cin >> ch;
    motionProxy.post.closeHand("RHand"); motionProxy.closeHand("LHand");


    std::vector<float> LArmPos = motionProxy.getPosition("LArm", 0, false);
    std::vector<float> RArmPos = motionProxy.getPosition("RArm", 0, false);

    //motionProxy.moveToward(0.1,0,0);
    usleep(2000000);
    std::cout << "NOW MOVE ARMS!\n";
    motionProxy.setMoveArmsEnabled(true, true);
    const int controlAxes = 7;
    const unsigned moveAxis = 0;
    for(unsigned i=0; i<3; ++i){
        LArmPos[moveAxis]+=0.005;
        RArmPos[moveAxis]-=0.005;
        motionProxy.positionInterpolations(
                                   AL::ALValue::array("LArm","RArm"),
                                   0,
                                   AL::ALValue::array(LArmPos,RArmPos),
                                   AL::ALValue::array(controlAxes,controlAxes),
                                   AL::ALValue::array(0.4f,0.4f));
    }
    for(unsigned i=0; i<6; ++i){
        LArmPos[moveAxis]-=0.005;
        RArmPos[moveAxis]+=0.005;
        motionProxy.positionInterpolations(
                                   AL::ALValue::array("LArm","RArm"),
                                   0,
                                   AL::ALValue::array(LArmPos,RArmPos),
                                   AL::ALValue::array(controlAxes,controlAxes),
                                   AL::ALValue::array(0.4f,0.4f));
    }
    for(unsigned i=0; i<3; ++i){
        LArmPos[moveAxis]+=0.005;
        RArmPos[moveAxis]-=0.005;
        motionProxy.positionInterpolations(
                                   AL::ALValue::array("LArm","RArm"),
                                   0,
                                   AL::ALValue::array(LArmPos,RArmPos),
                                   AL::ALValue::array(controlAxes,controlAxes),
                                   AL::ALValue::array(0.4f,0.4f));
    }
    motionProxy.setMoveArmsEnabled(false, false);
    usleep(2000000);
    motionProxy.stopMove();

    return 0;
}

// This main is for getting arm angles from a position make by user(human)
/*
int main(int argc, char **argv)
{
    std::string robotIp = "10.1.18.24";

    if (argc < 2) {
        std::cerr << "Usage: almotion_getangles robotIp "
                  << "(optional default \"169.254.67.213\")."<< std::endl;
    }
    else {
        robotIp = argv[1];
    }

    AL::ALMotionProxy motionProxy(robotIp);
    AL::ALRobotPostureProxy robotPostureProxy(robotIp);
    AL::ALBasicAwarenessProxy awarenessProxy(robotIp);

    motionProxy.setBreathEnabled("Body", false);
    motionProxy.setIdlePostureEnabled("Body", false);
    awarenessProxy.stopAwareness();

    // Example that finds the difference between the command and sensed angles.
    //robotPostureProxy.goToPosture("StandZero", 0.5f);
    motionProxy.stiffnessInterpolation(AL::ALValue::array("LArm","RArm"),
                                       0.0f, 0.75f);

    while(true){
        std::vector<float> rArmAngles = motionProxy.getAngles("RArm",false);
        std::cout << "RArm angles:" << rArmAngles << std::endl;
        std::vector<float> lArmAngles = motionProxy.getAngles("LArm",false);
        std::cout << "LArm angles:" << lArmAngles << std::endl;
    }

    return 0;
}
*/

