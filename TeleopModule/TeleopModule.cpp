#include "TeleopModule.h"

#include <qi/log.hpp>
#include <alvision/alimage.h>
#include <string>
#include <vector>
#include <iostream>

TeleopModule::TeleopModule(boost::shared_ptr<AL::ALBroker> broker,
                           const std::string& name)
    : AL::ALModule(broker, name),
      motionProxy(getParentBroker()),
      ttsProxy(getParentBroker()),
      currentMode(STOPPED),
      nextMode(STOPPED)
{
    lastOpCoords[0] = 0; lastOpCoords[1] = 0; lastOpCoords[2] = 0;
    lastOpCoords[3] = 0; lastOpCoords[4] = 0; lastOpCoords[5] = 0;

    // Describe the module here. This will appear on the webpage
    setModuleDescription("Teleoperation module.");

    functionName("startTeleop", getName(), "Starts the teleoperation");
    BIND_METHOD(TeleopModule::startTeleop);

    functionName("stopTeleop", getName(), "Stops the teleoperation");
    BIND_METHOD(TeleopModule::stopTeleop);

    functionName("changeMode", getName(), "Changes teleoperation mode.");
    addParam("newMode", "Selected mode,(walk,hands,wbhands");
    BIND_METHOD(TeleopModule::changeMode);

    functionName("setOpCoords",getName(),"Sets operator world coordinates");
    //setReturn("boolean", "return true");
    addParam("coordinates", "double vector [x y z roll pitch yaw]");
    BIND_METHOD(TeleopModule::setOpCoords);

    functionName("setOpInputsForCurMode",getName(),"Sets current inputs");
    addParam("inputs", "bool vector of keys, true is pushed and false"
                       " is released. Size may change for every mode");
    BIND_METHOD(TeleopModule::setOpInputsForCurMode);

    // If you had other methods, you could bind them here...
    /**
    * Bound methods can only take const ref arguments of basic types,
    * or AL::ALValue or return basic types or an AL::ALValue.
    */

    cs = new CameraServer("TeleopModule",AL::kVGA,AL::kRGBColorSpace,30,13333,
                          getParentBroker());
}

TeleopModule::~TeleopModule()
{
    delete cs;
    qiLogDebug("Teleoperation module destructed");
}

void TeleopModule::init()
{
    /**
   * Init is called just after construction.
   * Do something or not
   */
    ttsProxy.say("Teleoperation module initializing");
    qi::log::setSynchronousLog(true);
    qi::log::setVerbosity(qi::LogLevel_Debug);
    qiLogDebug("Teleoperation module initializing");
}


void TeleopModule::startTeleop()
{
    ttsProxy.say("Starting teleoperation");
    if(!actionThread.joinable())
        actionThread = boost::thread(&TeleopModule::actionThreadFunc, this);

}

void TeleopModule::stopTeleop()
{
    if(actionThread.joinable()){
        actionThread.interrupt();
        actionThread.join();
    }
    motionProxy.wbEnable(false);
    motionProxy.rest();
    ttsProxy.say("Teleoperation stopped");
}

void TeleopModule::changeMode(const AL::ALValue &newMode)
{
    nextMode = (TeleopMode)(int)newMode; // it works
}

// The caller should not call this function faster than 50 Hz
// as it wil useless because ALMotion runs in 50 Hz.
void TeleopModule::setOpCoords(const AL::ALValue &coordinates)
{
    boost::lock_guard<boost::mutex> guard(mtxOpCoords);
    for(int i=0; i<6; ++i)
        lastOpCoords[i] = coordinates[i];
}

void TeleopModule::setOpInputsForCurMode(const AL::ALValue &inputs)
{

}

// We can give real-time priority to this thread
// but is it necessary ?
void TeleopModule::actionThreadFunc()
{

    motionProxy.wakeUp();
    robotPostureProxy.goToPosture("StandInit", 0.5f);
    const int FRAME_ROBOT = 2, POSITION_AND_ROTATION = 63;
    const std::vector<float> initTfRArm =
                    motionProxy.getTransform("RArm", FRAME_ROBOT, false);
    const std::vector<float> initTfLArm =
                    motionProxy.getTransform("LArm", FRAME_ROBOT, false);

    while(true){
        if(currentMode != nextMode){
            // mode switch
            switch(nextMode){
            case STOPPED:

                break;
            case WALK:

                break;
            case HEAD:
                motionProxy.wbEnable(false);
                break;
            case CARTESIAN_HANDS:
                motionProxy.wbEnable(false);
                break;
            case CARTESIAN_HANDS_WB : // whole body
                motionProxy.wbEnable(true);
                motionProxy.wbFootState("Plane", "Legs");
                motionProxy.wbEnableBalanceConstraint(false, "Legs");
                break;
            }
            currentMode = nextMode;
        }

        switch(currentMode){
        case STOPPED:

            break;
        case WALK:

            break;
        case HEAD:

            break;
        case CARTESIAN_HANDS:
        case CARTESIAN_HANDS_WB:
            std::vector<float> targetTfRArm, targetTfLArm;
            targetTfRArm = initTfRArm;
            targetTfLArm = initTfLArm;
            {
                boost::lock_guard<boost::mutex> guard(mtxOpCoords);
                // the vector includes 12 variables which defines
                // a transformation matrix that defines effector
                // position and orientation
                // tf[0] tf[1] tf[2] tf[3](x)
                // tf[0] tf[1] tf[2] tf[7](y)
                // tf[0] tf[1] tf[2] tf[11](z)
                //   0     0     0      1
                // NEEDS CALIBRATION, but not here !!! at operator side
                targetTfRArm[3]  += lastOpCoords[0];
                targetTfRArm[7]  += lastOpCoords[1];
                targetTfRArm[11] += lastOpCoords[2];
                targetTfLArm[3]  += lastOpCoords[0];
                targetTfLArm[7]  += lastOpCoords[1];
                targetTfLArm[11] += lastOpCoords[2];
            }

            std::vector<std::string> effectorNames;
            effectorNames.resize(1u);
            effectorNames.at(0) = "RArm";
            //effectorNames.at(1) = "LArm";

            AL::ALValue pathList = AL::ALValue::array(
                        AL::ALValue::array(targetTfRArm));//,
                        //AL::ALValue::array(targetTfLArm));

            AL::ALValue timesList = AL::ALValue::array(
                    AL::ALValue::array(0.75f));//,   // RArm in seconds
                    //AL::ALValue::array(0.75f));  // LArm in seconds

            //motionProxy.setTransforms("RArm", FRAME_ROBOT, targetTfRArm,
            //                          0.5f, POSITION_AND_ROTATION);
            //motionProxy.setTransforms("LArm", FRAME_ROBOT, targetTfLArm,
            //                          0.5f, POSITION_AND_ROTATION);
            motionProxy.transformInterpolations(effectorNames, FRAME_ROBOT,
                                   pathList,POSITION_AND_ROTATION, timesList );
            break;
        }
        // this sleep is also an interruption point.
        // the sleeping time is 20 ms because ALMotion runs in
        // 50 Hz and we don't need to be faster than that
        // THAT SLEEPING TIME IS NOT ENOUGH FOR SETTRANSFORMS SO WE NEED TO SLEEP
        // SOMETHING LIKE 400 MS
        // OR DO NOT USE SETTRANSFORMS AND USE BLOCKING CALL, transformInterpolations
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    }

}
