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
      currentMode(STOP),
      nextMode(STOP),
      wholeBodySwitch(false)
{
    lastOpCoords[0] = 0; lastOpCoords[1] = 0; lastOpCoords[2] = 0;
    lastOpCoords[3] = 0; lastOpCoords[4] = 0; lastOpCoords[5] = 0;

    // Describe the module here. This will appear on the webpage
    setModuleDescription("Teleoperation module.");

    functionName("startTeleop", getName(), "Starts the teleoperation");
    BIND_METHOD(TeleopModule::startTeleop);

    functionName("stopTeleop", getName(), "Stops the teleoperation");
    BIND_METHOD(TeleopModule::stopTeleop);

    functionName("setMode", getName(), "Sets teleoperation mode.");
    addParam("newModes", "Selected mode");
    BIND_METHOD(TeleopModule::setMode);

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
    awareness.stopAwareness();
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
    awareness.startAwareness();
    ttsProxy.say("Teleoperation stopped");
}

void TeleopModule::setMode(const AL::ALValue &newMode)
{
    nextMode = (TeleopMode)(int)newMode;
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
    wholeBodySwitch = true;
}

// The caller should not call this function faster than 50 Hz
// as it wil useless because ALMotion runs in 50 Hz.
void TeleopModule::setOpCoords(const AL::ALValue &coordinates)
{
    boost::lock_guard<boost::mutex> guard(mtxOpCoords);
    std::cout << "setOpCoords, lastOpCoords: ";
    for(int i=0; i<6; ++i){
        lastOpCoords[i] = coordinates[i];
        std::cout << lastOpCoords[i] << " ";
    }
    std::cout << std::endl;
}

// We can give real-time priority to this thread
// but is it necessary ?
void TeleopModule::actionThreadFunc()
{
    motionProxy.wakeUp();
    // make sure joints is stiff enough to move
    motionProxy.stiffnessInterpolation("Head", 1.0f, 0.5f);
    motionProxy.stiffnessInterpolation("RArm", 1.0f, 0.5f);
    motionProxy.stiffnessInterpolation("LArm", 1.0f, 0.5f);

    robotPostureProxy.goToPosture("StandInit", 0.5f);
    const int FRAME_ROBOT = 2, POSITION = 7;
    const std::vector<float> initPosRArm =
            motionProxy.getPosition("RArm", FRAME_ROBOT, false);
    const std::vector<float> initPosLArm =
            motionProxy.getPosition("LArm", FRAME_ROBOT, false);
    std::vector<float> targetPosLArm, targetPosRArm;
    std::vector<std::string> effectorNames;
    AL::ALValue armsPathList;
    AL::ALValue timesList;

    while(true){
        effectorNames.clear();
        armsPathList.clear();
        timesList.clear();

        if(currentMode != nextMode){
            currentMode = nextMode;
            std::cout << "Mode switched to:" << currentMode << std::endl;
        }

        static bool  useWholeBody = false;
        if(wholeBodySwitch){
            useWholeBody=!useWholeBody;
            if(useWholeBody){
                motionProxy.wbEnable(true);
                motionProxy.wbFootState("Plane", "Legs"); // can be also Plane Legs
                motionProxy.wbEnableBalanceConstraint(false, "Legs");
            }
            else{
                motionProxy.wbEnable(false);
            }
            wholeBodySwitch = false;
        }

        // for the problem of watching hands while hand teloperation
        // involves controlling head movements. To solve this problem,
        // the pitch and roll of operator will be used to control
        // head pitch and yaw.
        // BETTER SOLUTION, USE BOTTOM CAMERA !

        if(currentMode == HEAD){
            std::vector<std::string> effectorNames;
            effectorNames.push_back("HeadPitch"); // -0.330041 to 0.200015
            effectorNames.push_back("HeadYaw"); // -2.0857 to 2.0857

            AL::ALValue targetHeadAngles;
            {
                boost::lock_guard<boost::mutex> guard(mtxOpCoords);
                // The variables define head angles
                targetHeadAngles = AL::ALValue::array(
                        lastOpCoords[4],  // Pitch
                        lastOpCoords[3]); // Yaw (haptic roll)
            }
            std::cout << "targetHeadAngles[0]:"
                      << targetHeadAngles[0] << "\t\tlastOpCoords[4]:"
                      << lastOpCoords[4] << std::endl;
            std::cout << "targetHeadAngles[1]:"
                      << targetHeadAngles[1] << "\t\tlastOpCoords[3]:"
                      << lastOpCoords[3] << std::endl << std::endl;

            timesList = AL::ALValue::array( 0.5f, 0.5f);

            if(motionProxy.areResourcesAvailable(effectorNames))
                motionProxy.angleInterpolation(effectorNames,targetHeadAngles,
                                           timesList,true);
        }
        else if(currentMode == LLEG){

        }
        else if(currentMode == RLEG){

        }
        else if(currentMode == WALK){

        }
        else if(currentMode == LARM || currentMode == RARM ||
                currentMode == BOTH_ARMS)
        {
            targetPosLArm = std::vector<float>(initPosLArm);
            targetPosRArm = std::vector<float>(initPosRArm);
            {
                boost::lock_guard<boost::mutex> guard(mtxOpCoords);
                for(int i=0; i<5; ++i){
                    std::cout << "targetPosLArm[" << i <<"]: " << targetPosLArm[i];
                    targetPosLArm[i] += lastOpCoords[i];
                    targetPosRArm[i] += lastOpCoords[i];
                    std::cout << " -> " << targetPosLArm[i] << std::endl;
                }
                std::cout << std::endl;
            }



            if(currentMode == LARM || currentMode == BOTH_ARMS){
                armsPathList.arrayPush(targetPosLArm);
                timesList.arrayPush(1.0f);
                effectorNames.push_back("LArm");
            }
            if(currentMode == RARM || currentMode == BOTH_ARMS){
                armsPathList.arrayPush(targetPosRArm);
                timesList.arrayPush(1.0f);
                effectorNames.push_back("RArm");
            }

            if(motionProxy.areResourcesAvailable(effectorNames))
                motionProxy.positionInterpolations(effectorNames, FRAME_ROBOT,
                                            armsPathList,4, timesList );


            // [RL]WristYaw range: -1.8238 to 1.8238 radians
            /*if(currentMode == RARM)
                motionProxy.angleInterpolationWithSpeed(
                            "RWristYaw",targetPosRArm[3],0.4f);
            else if(currentMode == LARM)
                motionProxy.angleInterpolationWithSpeed(
                            "LWristYaw",-targetPosLArm[3],0.4f);*/

        }
        // this sleep is also an interruption point.
        // the sleeping time is 20 ms because ALMotion runs in
        // 50 Hz and we don't need to be faster than that
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
        // THAT SLEEPING TIME IS NOT ENOUGH FOR SETTRANSFORMS
        // SO WE NEED TO SLEEP
        // SOMETHING LIKE 400 MS
        // OR DO NOT USE SETTRANSFORMS AND USE BLOCKING CALL,
        // transformInterpolations so this way we won't need a sleep
        boost::this_thread::interruption_point();
    } // while(true)

}
