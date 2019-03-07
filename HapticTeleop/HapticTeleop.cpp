
#include "HapticTeleop.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <math.h>
#ifndef M_PI
 #define M_PI 3.14159265358979323846
#endif

#define DEBUG


/**
 * This function is called when the control program is loaded to zenom.
 * Use this function to register control parameters, to register log variables
 * and to initialize control parameters.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::initialize()
{
    // connect to nao
    broker = AL::ALBroker::createBroker("MyBroker", "", 0, NAO_IP_ADDR, 9559);

    motionProxy = boost::shared_ptr<AL::ALMotionProxy>(
                new AL::ALMotionProxy(broker));
    motionProxy->setBreathEnabled("Body", false);
    motionProxy->setIdlePostureEnabled("Body", false);

    awarenessProxy = boost::shared_ptr<AL::ALBasicAwarenessProxy>(
                new AL::ALBasicAwarenessProxy(broker));
    awarenessProxy->stopAwareness();

    memoryProxy =  boost::shared_ptr<AL::ALMemoryProxy>(
                new AL::ALMemoryProxy(broker));

    automoProxy = boost::shared_ptr<AL::ALAutonomousMovesProxy>(
                new AL::ALAutonomousMovesProxy(broker));
    automoProxy->setBackgroundStrategy("none");

    //postureProxy = boost::shared_ptr<AL::ALRobotPostureProxy>(
    //            new AL::ALRobotPostureProxy(broker));

    registerLogVariable( w.getElementsPointer(), "w", 1, HAPTIC_AXES );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, HAPTIC_AXES );
    registerLogVariable( F.getElementsPointer(), "Haptic Force", 1,HAPTIC_AXES);
    registerLogVariable( &manipulability, "Manipulability", 1, 1);

    registerControlVariable( &modeSelectedByUser, "_selected_mode",  1, 1);
    registerControlVariable( &rHandStateFromUser, "_rightHandState", 1, 1);
    registerControlVariable( &lHandStateFromUser, "_leftHandState",  1, 1);
    registerControlVariable( &wbStateFromUser,    "_wholeBodyState", 1, 1);
    registerControlVariable( &distBetwArms,       "_distBetwArms",   1, 1);

    modeSelectedByUser = STOP;
    rHandStateFromUser = lHandStateFromUser = wbStateFromUser = 0;
    distBetwArms = 0.05;

    commTask = new NetworkTask(this);
    commTask->runTask();

    return 0;
}

/**
 * This function is called when the START button is pushed from zenom.
 *
 * @return If you return 0, the control starts and the doloop() function is
 * called periodically. If you return nonzero, the control will not start.
 */
int HapticTeleop::start()
{

    hapticWand.openDevice();        // Open the q8 card
    hapticWand.calibrateWand();     // Calibrate the haptic wand
    hapticWand.enableWand();
    enableForceIfDisabled(1.0);

    // The program will still run if connection is unsucccessful
    connectToMATLAB();

    curFbMode = newFbMode = NO_FEEDBACK_CNT;
    w_temp = 0,HAPTIC_START_YPOS,0,0,0;
    for(int i=0; i<MANIP_MODES; ++i)
        lastSamples[i]= w_temp;
    wd = 0,HAPTIC_START_YPOS,0,0,0;

    ColumnVector<5> firstSample;
    firstSample =
            hapticWand.firstSample()[0],
            hapticWand.firstSample()[1],
            hapticWand.firstSample()[2],
            hapticWand.firstSample()[3],
            hapticWand.firstSample()[4];
    wd_inter = firstSample;
    positionController.reset( firstSample, period() );

    motionProxy->wakeUp();
    // make sure joints are stiff enough to move
    motionProxy->stiffnessInterpolation(
                AL::ALValue::array("Head","LArm","RArm"), 1.0f, 0.5f);
    // Go to predefined pose, left arm + right arm
    preArmAnglesVec.assign(predefinedArmAngles,
                           predefinedArmAngles+NUM_OF_ARM_ANGLES*2);
    motionProxy->angleInterpolationWithSpeed( AL::ALValue::array("LArm","RArm"),
                                              preArmAnglesVec,
                                              0.5f);

    initTfRArm = AL::Math::Transform(
                motionProxy->getTransform("RArm", FRAME_TORSO, false));
    initTfLArm = AL::Math::Transform(
                motionProxy->getTransform("LArm", FRAME_TORSO, false));

    curTeleopMode = STOP;

    return 0;
}


/**
 * This function is called periodically (as specified by the control frequency).
 * The useful functions that you can call used in doloop() are listed below.
 *
 * frequency()          returns frequency of simulation.
 * period()             returns period of simulation.
 * duration()           returns duration of simulation.
 * simTicks()           returns elapsed simulation ticks.
 * simTimeInNano()      returns elapsed simulation time in nano seconds.
 * simTimeInMiliSec()   returns elapsed simulation time in miliseconds.
 * simTimeInSec()       returns elapsed simulation time in seconds.
 * overruns()           returns the count of overruns.
 *
 * @return If you return 0, the control will continue to execute. If you return
 * nonzero, the control will abort and stop() function will be called.
 */
int HapticTeleop::doloop()
{
    double jointAngles[HAPTIC_NUM_OF_JOINTS]; // joint angles in radians
    hapticWand.jointAngles( jointAngles );
    // current world position.
    hapticWand.forwardKinematics( jointAngles, w.getElementsPointer() );

    static unsigned operatorInputTimeCounter = 0;
    if(++operatorInputTimeCounter * period() == INPUT_CHECK_PERIOD_SEC)
    {
        if( commDataMutex.try_lock()){
            if(commQueue.empty()){
                checkInputAndSetMode();
                if(curTeleopMode != STOP)
                    moveRobot();
            }
            commDataMutex.unlock();
            commCondVar.notify_one();
            operatorInputTimeCounter = 0;
        }
        else{
            //couldn't lock, try again next period
            operatorInputTimeCounter = 0;
        }
#ifdef DEBUG
                std::cerr << "Teleop mode:" << curTeleopMode <<
                             " Feedback mode:" << curFbMode << " Cooldown:" <<
                             fbCooldown << " hapticWandForceEnabled:" <<
                             hapticWandForceEnabled << std::endl << std::flush;
#endif
    }

    FeedbackMode fbm = newFbMode;
    // state machine !
    if((curFbMode == FEEDBACK_CNT && fbm == NO_FEEDBACK_START) ||
       (curFbMode == NO_FEEDBACK_CNT && fbm != NO_FEEDBACK_START))
    {
        curFbMode = fbm;
    }

    switch (curFbMode) {
    case NO_FEEDBACK_START:
        wd = wd_inter = w;
        curFbMode = NO_FEEDBACK_CNT;
        break;
    case FEEDBACK_START:
        enableForceIfDisabled(period()*5);
        if(isArmMode(curTeleopMode))
            wd = 0,HAPTIC_START_YPOS,0,0,0;
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;
        wd_inter = w;
        curFbMode = FEEDBACK_CNT;
        break;
    /*case TILT_START:
        enableForce();
        wd = w_temp = wd_inter = w;
        fbMode = TILT_CONTINUE;
        break;
     */
    default:
        break;
    }

    switch (curFbMode) {
    case FEEDBACK_CNT:
        wd_inter = setPoint.find_wd(wd, wd_inter, period());
        break;
    case NO_FEEDBACK_CNT:
        buttonState = !hapticWand.readDigital(HAPTIC_BUTTON_PIN);
        if(!buttonState) {
            enableForceIfDisabled(period()*10);
            wd_inter = setPoint.find_wd(wd, wd_inter, period());
        }
        else{
            disableForce();
            wd = wd_inter = w;
        }
        break;
    /*case TILT_CONTINUE:
        wd(3) = w_temp(3) + 0.004 * cos(13.5*M_PI*elapsedTime()) +
                0.02 * sin(18*M_PI*elapsedTime());
        break;
    */
    default:
        break;
    }

    // Desired position.


    if(hapticWandForceEnabled){
        F = positionController.force( w, wd_inter ) / forceDivider;
        if(forceDivider > 1){
            forceDivider -= forceDividerStep;
        }
        else
            forceDivider = 1;
    }
    else
        F = 0,0,0,0,0;

    hapticWand.generateForces( period(), jointAngles, F.getElementsPointer());

    return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::stop()
{
    enableForceIfDisabled(1.0);


    //Go to start position of haptic wand
    wd = 0,0,0,0,0;
    wd.setElement(2, HAPTIC_START_YPOS);
    std::cout << "going to start ypos" << std::endl;
    hapticGoToPosBlocking(0.05);
    wd = 0,0.124,0.03,0,0;
    std::cout << "going to init pos" << std::endl;
    hapticGoToPosBlocking(0.02);
    std::cout << "now disabling wand" << std::endl;

    disableForce();
    hapticWand.closeDevice();

    awarenessProxy->startAwareness();
    motionProxy->wbEnable(false);
    motionProxy->rest();

    if(matlabTCPSocket != -1)
        disconnectFromMATLAB();

    std::cout << "stopped\n";

    return 0;
}


/**
 * This function is called when the control is unloaded. It happens when
 * the user loads a new control program or exits.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::terminate()
{

    commTask->finishTask();
    delete commTask;

    hapticWand.disableWand();

    return 0;
}

void HapticTeleop::checkInputAndSetMode()
{
    // don't let it to switch whole body while not stopped
    if(wbState != (bool)wbStateFromUser){
        wbState = (bool)wbStateFromUser;
        commQueue.push([this](){
            if(wbState){
                motionProxy->wbEnable(true);
                motionProxy->wbFootState("Plane", "Legs");
                motionProxy->post.wbEnableBalanceConstraint(true, "Legs");
            }
            else{
                motionProxy->post.wbEnable(false);
            }
        });
    }

    if(rHandState != (bool)rHandStateFromUser){
        rHandState = (bool)rHandStateFromUser;
        commQueue.push([this](){
            openOrCloseHand(rHandState,"RHand");
        });
    }

    if(lHandState != (bool)lHandStateFromUser){
        lHandState = (bool)lHandStateFromUser;
        commQueue.push([this](){
            openOrCloseHand(lHandState,"LHand");
        });
    }

    // mode change
    TeleopMode newMode = (TeleopMode)modeSelectedByUser;

    // mode switching can only be done while button is released
    if(!buttonState && (curTeleopMode != newMode)){
        if(isArmMode(curTeleopMode))
            lastSamples[curTeleopMode] = w;
        if(isArmMode(newMode))
            wd = lastSamples[newMode];
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;

        wd_inter = w;

        curTeleopMode = newMode;
        fbCooldown = false;
        newFbMode = NO_FEEDBACK_START;
        std::cout << "Mode switched to " << curTeleopMode << std::endl;
        commQueue.push([this](){
            motionProxy->stopMove();
            /*
            motionProxy->angleInterpolationWithSpeed(
                        AL::ALValue::array("LArm","RArm"),
                        preArmAnglesVec,
                        0.5f);
            openOrCloseHand(rHandState,"RHand");
            openOrCloseHand(lHandState,"LHand");
            */
        });
    }
}

void HapticTeleop::moveRobot()
{
    // Op X is Nao Y, Op Y is Nao X
    std::vector<float> opCoords(HAPTIC_AXES);
    opCoords[0] = ROUND(((isArmMode(curTeleopMode) ?
            HAPTIC_START_YPOS : HAPTIC_MID_YPOS)-w(2))
            * mappingCoefs[curTeleopMode][1] + mappingBias[curTeleopMode][1]);
    opCoords[1] = ROUND(w(1) * mappingCoefs[curTeleopMode][0] +
            mappingBias[curTeleopMode][0]);
    opCoords[2] = ROUND(w(3) * mappingCoefs[curTeleopMode][2] +
            mappingBias[curTeleopMode][2]);
    opCoords[3] = ROUND(w(5) * mappingCoefs[curTeleopMode][4] +
            mappingBias[curTeleopMode][4]);
    opCoords[4] = ROUND(w(4) * mappingCoefs[curTeleopMode][3]+
            mappingBias[curTeleopMode][3]);

    commQueue.push([=]() mutable {
        //std::cout << "opCoords:" << opCoords << std::endl;
        using namespace AL;

        if (curTeleopMode == BOTH_ARMS){
            if(matlabTCPSocket != -1){
                opCoords[4] = distBetwArms; // race condition
                // send opCoords and current mode to MATLAB
                opCoords.push_back(BOTH_ARMS);
                write(matlabTCPSocket,(char*)opCoords.data(),
                      opCoords.size()*sizeof(float));
                // receive target angles and feedback mode from matlab
                std::vector<float> armAngles(2*NUM_OF_ARM_ANGLES+1);
                read(matlabTCPSocket,(char*)armAngles.data(),
                        armAngles.size()*sizeof(float));
                manipulability = (double)armAngles.back();

                // move arms of nao
                armAngles.pop_back();
                motionProxy->setAngles(ALValue::array("LArm","RArm"),
                                       armAngles,0.9f);

                // Feedback
                if(fbCooldown && elapsedFbTime() > FB_TOTAL_TIME)
                {
                    fbCooldown = false;
                }

                if(curFbMode == FEEDBACK_CNT && elapsedFbTime() < FB_TOTAL_TIME
                        && elapsedFbTime() >= FB_IMPOSE_TIME)
                {
                    newFbMode = NO_FEEDBACK_START;
                    fbCooldown = true;
                }

                if(curFbMode == NO_FEEDBACK_CNT && !fbCooldown &&
                        (float)manipulability > manipThreshold)
                {
                    fbStartTime = elapsedTime();
                    newFbMode = FEEDBACK_START;
                }
            }
            else{
                std::cout << "Warning: BOTH ARMS mode can be only used when "
                          << "connected to MATLAB script.\n";
            }
        }
        else if(curTeleopMode == RARM || curTeleopMode == LARM){
            Math::Transform targetTfArm = Math::Transform::fromPosition(
                        opCoords[0],opCoords[1],opCoords[2],
                        (curTeleopMode == RARM ? -1 : 1) * opCoords[3],0,0) *
                        (curTeleopMode == RARM ? initTfRArm : initTfLArm);
            motionProxy->transformInterpolations(
                       (curTeleopMode == RARM? "RArm" : "LArm"),
                       FRAME_TORSO, targetTfArm.toVector(),
                       CONTROL_AXES,0.5f);

            // Feedback
            if( matlabTCPSocket != -1)
            {  // get manipulability
                auto armAngles = motionProxy->getAngles(
                            (curTeleopMode == RARM ? "RArm" : "LArm"),false);
                armAngles[5] = curTeleopMode;
                write(matlabTCPSocket,(char*)armAngles.data(),
                      armAngles.size()*sizeof(float));
                float manip;
                read(matlabTCPSocket,(char*)&manip,sizeof(float));
                manipulability = manip;
            }

            if(fbCooldown && elapsedFbTime() > FB_TOTAL_TIME)
            {
                fbCooldown = false;
            }

            if(curFbMode == FEEDBACK_CNT && elapsedFbTime() < FB_TOTAL_TIME &&
                    elapsedFbTime() >= FB_IMPOSE_TIME)
            {
                newFbMode = NO_FEEDBACK_START;
                fbCooldown = true;
            }

            if(curFbMode == NO_FEEDBACK_CNT && !fbCooldown &&
                    (float)manipulability > manipThreshold)
            {
                fbStartTime = elapsedTime();
                newFbMode = FEEDBACK_START;
            }
        }

        else if(curTeleopMode == HEAD){
            ALValue effectorNames = ALValue::array("HeadPitch", "HeadYaw");

            // The variables define head angles
            ALValue targetHeadAngles = ALValue::array(
                        opCoords[4],  // Pitch
                        opCoords[3]); // Yaw (haptic roll)

            motionProxy->angleInterpolationWithSpeed(effectorNames,
                                                     targetHeadAngles,0.3f);
        }
        else if(curTeleopMode == WALK_TOWARD){
            if(opCoords[0] > 0.2 && areOfAnyFeetBumpersPressed()){
                motionProxy->stopMove();
                newFbMode = FEEDBACK_START; // override
            }
            else {
                if(opCoords[0] >= -0.2 && opCoords[0] <= 0.2 &&
                    opCoords[1] >= -0.2 && opCoords[1] <= 0.2)
                { // stop
                    motionProxy->stopMove();
                }
                else if (opCoords[0] > 0.2 || opCoords[0] < -0.2){
                    // forward or backward
                    motionProxy->moveToward(opCoords[0], 0, opCoords[1]);
                }
                else if (opCoords[1] > 0.2 || opCoords[1] < -0.2){
                    // walk horizontal (crab walk :))
                    motionProxy->moveToward(0, opCoords[1], 0);
                }
                newFbMode = NO_FEEDBACK_START;
            }

        }
    });

}

void HapticTeleop::openOrCloseHand(bool isOpen, string hand)
{
    if(isOpen)
        motionProxy->openHand(hand);
    else
        motionProxy->closeHand(hand);
}

inline void HapticTeleop::enableForceIfDisabled(double dividorStep)
{
    if(!hapticWandForceEnabled){
        hapticWandForceEnabled = true;
        forceDivider = 10;
        forceDividerStep = dividorStep;
    }
}

inline void HapticTeleop::disableForce()
{
    hapticWandForceEnabled = false;
}

// go to target wd and stop when wand is close as threshold cm
void HapticTeleop::hapticGoToPosBlocking(double threshold)
{
    double jointAngles[HAPTIC_NUM_OF_JOINTS]; // joint angles in radians
    hapticWand.jointAngles( jointAngles );
    // current world position.
    hapticWand.forwardKinematics( jointAngles, w.getElementsPointer() );
    wd_inter = w;

    while(ColumnVector<5>((w-wd)).norm() > threshold){
        wd_inter = setPoint.find_wd(wd, wd_inter, period() );
        ColumnVector<5> F = positionController.force( w, wd_inter );
        hapticWand.generateForces(period(), jointAngles, F.getElementsPointer());
        hapticWand.jointAngles( jointAngles );
        // current world position.
        hapticWand.forwardKinematics( jointAngles, w.getElementsPointer() );
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void HapticTeleop::connectToMATLAB()
{
    // connect to matlab
    struct sockaddr_in serv_addr;
    struct hostent *server;

    matlabTCPSocket = socket(AF_INET, SOCK_STREAM, 0);
    server = gethostbyname("localhost");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(MATLAB_TCP_PORT);
    if(connect(matlabTCPSocket,(struct sockaddr *)&serv_addr,sizeof(serv_addr))
            == -1)
    {
        perror("Couldn't connect to matlab, running without matlab\n");

        close(matlabTCPSocket);
        matlabTCPSocket = -1;
    }
    else{
        /*if(write(matlabTCPSocket, (char*)&manipThreshold, sizeof(float)) == -1){
            perror("Couldn't write manipThreshold to matlab\n");
        }*/
    }
}

void HapticTeleop::disconnectFromMATLAB()
{
    shutdown(matlabTCPSocket,SHUT_RDWR);
    close(matlabTCPSocket);
}

bool HapticTeleop::areOfAnyFeetBumpersPressed()
{
    AL::ALValue keys = AL::ALValue::array(
        "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
        "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
        "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
        "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value");
    AL::ALValue data = memoryProxy->getListData(keys);
    //std::cout << "Bumpers: " << (float)data[0] << " " << (float)data[1] << " "
    //          << (float)data[2] << " " << (float)data[3] << std::endl;
    return ((float)data[0]) || ((float)data[1]) ||
            ((float)data[2]) || ((float)data[3]);

}
