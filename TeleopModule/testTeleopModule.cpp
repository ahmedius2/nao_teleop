#include <iostream>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include <cmath>

using namespace AL::Math;

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

    motionProxy.wbEnable(true);
    motionProxy.wbFootState("Free", "Legs"); // can be also Plane Legs
    motionProxy.wbEnableBalanceConstraint(false, "Legs");

    // Example that finds the difference between the command and sensed angles.
    robotPostureProxy.goToPosture("StandZero", 0.5f);

    const float pi = 3.14159265359f;
    const int FRAME_TORSO = 1, CONTROL_AXES = 63;
    //AL::Math::Transform headTf = motion.getTransform("Head", 1, false);
    static const float armAngles[] = {
        1.38503, -0.0193124, -1.54517, -1.37153, 0.0280995, 0,
        1.38503, 0.0193124, 1.54517, 1.37153, -0.0280997, 0};
    std::vector<float>targetArmAngles;
    targetArmAngles.assign(armAngles,armAngles+12);
    motionProxy.angleInterpolationWithSpeed( AL::ALValue::array("LArm","RArm"),
                                             targetArmAngles,
                                             0.3f);
    /*
    const Transform initTfRArm = Transform(
                motionProxy.getTransform("RArm", FRAME_TORSO, false));
    const Transform initTfLArm = Transform(
                motionProxy.getTransform("LArm", FRAME_TORSO, false));
    */
    float xCenter = 0.15f;
    float yCenter = 0.0f;
    float zCenter = 0.02f;
    float r = 0.05;

    for(float alpha=-pi/3.0; alpha<=pi/3.0 ; alpha += 0.1)
    {
        // TO DO
        // initTfArmsCenter
        Transform targetTfLArm = Transform(xCenter, yCenter+(r*std::cos(alpha)),
                          zCenter-(r*std::sin(alpha))) *
                Transform::fromRotX(-alpha-pi/2.0f);
        Transform targetTfRArm = Transform(xCenter, yCenter-(r*std::cos(alpha)),
                          zCenter+(r*std::sin(alpha))) *
                Transform::fromRotX(-alpha+pi/2.0f);

        std::cout << "Transform of LArm" << std::endl;
        std::cout << targetTfLArm.r1_c1 << " " << targetTfLArm.r1_c2 << " " <<
                     targetTfLArm.r1_c3 << " " << targetTfLArm.r1_c4 << std::endl;
        std::cout << targetTfLArm.r2_c1 << " " << targetTfLArm.r2_c2 << " " <<
                     targetTfLArm.r2_c3 << " " << targetTfLArm.r2_c4 << std::endl;
        std::cout << targetTfLArm.r3_c1 << " " << targetTfLArm.r3_c2 << " " <<
                     targetTfLArm.r3_c3 << " " << targetTfLArm.r3_c4 << std::endl;
        std::cout << "0 0 0 1" << std::endl;
        std::cout << "Transform of RArm" << std::endl;
        std::cout << targetTfRArm.r1_c1 << " " << targetTfRArm.r1_c2 << " " <<
                     targetTfRArm.r1_c3 << " " << targetTfRArm.r1_c4 << std::endl;
        std::cout << targetTfRArm.r2_c1 << " " << targetTfRArm.r2_c2 << " " <<
                     targetTfRArm.r2_c3 << " " << targetTfRArm.r2_c4 << std::endl;
        std::cout << targetTfRArm.r3_c1 << " " << targetTfRArm.r3_c2 << " " <<
                     targetTfRArm.r3_c3 << " " << targetTfRArm.r3_c4 << std::endl;
        std::cout << "0 0 0 1" << std::endl;
        AL::ALValue armsPathList = AL::ALValue::array(
                    AL::ALValue(targetTfLArm.toVector()),
                    AL::ALValue(targetTfRArm.toVector()));
        AL::ALValue axisMask = AL::ALValue::array(CONTROL_AXES,CONTROL_AXES);
        AL::ALValue timesList = AL::ALValue::array(
                    AL::ALValue(0.5f),
                    AL::ALValue(0.5f));
        AL::ALValue effectorNames = AL::ALValue::array("LArm","RArm");

        motionProxy.transformInterpolations(effectorNames, FRAME_TORSO,
                                            armsPathList,axisMask, timesList);
    }

    motionProxy.rest();

    return 0;
}

/*
int main(int argc, char **argv)
{
  std::string robotIp = "10.1.18.5";

  if (argc < 2) {
    std::cerr << "Usage: almotion_getbodynames robotIp "
              << "(optional default \"127.0.0.1\")."<< std::endl;
  }
  else {
    robotIp = argv[1];
  }

  AL::ALMotionProxy motion(robotIp);

  // Example showing how to get the names of all the joints in the body.
  std::vector<std::string> bodyNames = motion.getBodyNames("Body");
  std::cout << "All joints in Body: " << std::endl;
  std::cout << bodyNames << std::endl;

  // Example showing how to get the names of all the joints in the left leg.
  std::vector<std::string> leftLegJointNames = motion.getBodyNames("LLeg");
  std::cout << "All joints in LLeg: " << std::endl;
  std::cout << leftLegJointNames << std::endl;

  return 0;
}
*/
