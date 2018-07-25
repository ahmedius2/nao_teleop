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
      currentModes(0),
      nextModes(0)
{
    lastOpCoords[0] = 0; lastOpCoords[1] = 0; lastOpCoords[2] = 0;
    lastOpCoords[3] = 0; lastOpCoords[4] = 0; lastOpCoords[5] = 0;

    // Describe the module here. This will appear on the webpage
    setModuleDescription("Teleoperation module.");

    functionName("startTeleop", getName(), "Starts the teleoperation");
    BIND_METHOD(TeleopModule::startTeleop);

    functionName("stopTeleop", getName(), "Stops the teleoperation");
    BIND_METHOD(TeleopModule::stopTeleop);

    functionName("setModes", getName(), "Sets teleoperation modes.");
    addParam("newModes", "Selected modes, stored in a single int by ORing");
    BIND_METHOD(TeleopModule::setModes);

    functionName("openOrCloseRHand", getName(), "Changes right hand state.");
    BIND_METHOD(TeleopModule::openOrCloseRHand);

    functionName("openOrCloseLHand", getName(), "Changes left hand state.");
    BIND_METHOD(TeleopModule::openOrCloseLHand);

    functionName("switchWholeBody", getName(), "Use whole body or not.");
    BIND_METHOD(TeleopModule::switchWholeBody);

    functionName("setOpCoords",getName(),"Sets operator world coordinates");
    addParam("coordinates", "double vector [x y z roll pitch yaw]");
    BIND_METHOD(TeleopModule::setOpCoords);


    // If you had other methods, you could bind them here...
    /**
    * Bound methods can only take const ref arguments of basic types,
    * or AL::ALValue or return basic types or an AL::ALValue.
    */

    cs = new CameraServer("TeleopModule",AL::kQVGA,AL::kRGBColorSpace,30,13333,
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
    //qi::log::setVerbosity(qi::LogLevel_Debug);
    qiLogInfo("Teleoperation module initializing");
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

void TeleopModule::setModes(const AL::ALValue &newModes)
{
    nextModes = (int)newModes;
}

void TeleopModule::openOrCloseRHand()
{
    static bool rHandOpen=true;
    if(rHandOpen)
        motionProxy.openHand("RHand");
    else
        motionProxy.closeHand("RHand");
    rHandOpen = !rHandOpen;
}

void TeleopModule::openOrCloseLHand()
{
    static bool lHandOpen=true;
    if(lHandOpen)
        motionProxy.openHand("LHand");
    else
        motionProxy.closeHand("LHand");
    lHandOpen = !lHandOpen;
}

void TeleopModule::switchWholeBody()
{
    static bool use=true;
    if(use){
        motionProxy.wbEnable(true);
        motionProxy.wbFootState("Plane", "Legs");
        motionProxy.wbEnableBalanceConstraint(false, "Legs");
    }
    else{
        motionProxy.wbEnable(false);
    }
    use = !use;
}

// The caller should not call this function faster than 50 Hz
// as it wil useless because ALMotion runs in 50 Hz.
void TeleopModule::setOpCoords(const AL::ALValue &coordinates)
{
    boost::lock_guard<boost::mutex> guard(mtxOpCoords);
    for(int i=0; i<6; ++i)
        lastOpCoords[i] = coordinates[i];
}

// We can give real-time priority to this thread
// but is it necessary ?
void TeleopModule::actionThreadFunc()
{
    motionProxy.wakeUp();
    robotPostureProxy.goToPosture("StandInit", 0.5f);
    const int FRAME_ROBOT = 2, POSITION_AND_ROTATION = 31; // don't control wz
    const std::vector<float> initPosRArm =
                    motionProxy.getPosition("RArm", FRAME_ROBOT, false);
    const std::vector<float> initPosLArm =
                    motionProxy.getPosition("LArm", FRAME_ROBOT, false);

    while(true){

        if(currentModes != nextModes){
            currentModes = nextModes;
            std::cout << "Mode switched to:" << currentModes << std::endl;
        }

        // for the problem of watching hands while hand teloperation
        // involves controlling head movements. To solve this problem,
        // the pitch and roll of operator will be used to control
        // head pitch and yaw.

        if(currentModes & WALK){

        }

        if(currentModes & HEAD){
            AL::ALValue targetHeadAngles;
            {
                boost::lock_guard<boost::mutex> guard(mtxOpCoords);
                // The variables define head angles
                targetHeadAngles = AL::ALValue::array(
                           lastOpCoords[3],  // Pitch
                           lastOpCoords[4]); // Yaw
            }

            std::vector<std::string> effectorNames;
            effectorNames.push_back("HeadPitch");
            effectorNames.push_back("HeadYaw");

             motionProxy.angleInterpolationWithSpeed(effectorNames,
                                targetHeadAngles, 0.3f);
        }
        else if(currentModes & (CARTESIAN_RHAND | CARTESIAN_LHAND)){
            std::vector<float> targetPosRArm(initPosRArm),
                    targetPosLArm(initPosLArm);
            {
                boost::lock_guard<boost::mutex> guard(mtxOpCoords);
                // the vector includes 12 variables which defines
                // a transformation matrix that defines effector
                // position and orientation
                // tf[0] tf[1] tf[2] tf[3](x)
                // tf[0] tf[1] tf[2] tf[7](y)
                // tf[0] tf[1] tf[2] tf[11](z)
                //   0     0     0      1
                for(int i=0; i<6; ++i){
                    targetPosRArm[i] += lastOpCoords[i];
                    targetPosLArm[i] += lastOpCoords[i];
                }
            }

            std::vector<std::string> effectorNames;
            AL::ALValue armsPathList;
            AL::ALValue timesList;
            if(currentModes & CARTESIAN_RHAND){
                armsPathList.arrayPush(targetPosRArm);
                timesList.arrayPush(0.3f);
                effectorNames.push_back("RArm");
            }
            if(currentModes & CARTESIAN_LHAND){
                armsPathList.arrayPush(targetPosLArm);
                timesList.arrayPush(0.3f);
                effectorNames.push_back("LArm");
            }

            // blocking call
            motionProxy.positionInterpolations(effectorNames, FRAME_ROBOT,
                                armsPathList,POSITION_AND_ROTATION, timesList );
        }
        else if(currentModes & CARTESIAN_LLEG){

        }
        else if(currentModes & CARTESIAN_RLEG){

        }
        // this sleep is also an interruption point.
        // the sleeping time is 20 ms because ALMotion runs in
        // 50 Hz and we don't need to be faster than that
        // THAT SLEEPING TIME IS NOT ENOUGH FOR SETTRANSFORMS SO WE NEED TO SLEEP
        // SOMETHING LIKE 400 MS
        // OR DO NOT USE SETTRANSFORMS AND USE BLOCKING CALL, transformInterpolations
        // so this way we won't need a sleep
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
        if(currentModes == 0) // no mode is active, sleep
            boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    }

}
