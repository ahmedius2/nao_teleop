
#include "HapticTeleop.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <math.h>
#ifndef M_PI
 #define M_PI 3.14159265358979323846
#endif

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

    registerLogVariable( w.getElementsPointer(), "w", 1, HAPTIC_AXES );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, HAPTIC_AXES );
    //registerLogVariable( wd_inter.getElementsPointer(), "wd_inter", 1,
    //                     HAPTIC_AXES );

    //registerControlVariable( &setPoint.v_lim_mult, "v_lim_mult",1,1);
    registerControlVariable( &openCloseLHandKey, "key_q", 1, 1);
    registerControlVariable( &headKey, "key_w", 1, 1);
    registerControlVariable( &openCloseRHandKey, "key_e", 1, 1);
    registerControlVariable( &cartLHandKey, "key_a", 1, 1);
    registerControlVariable( &walkToKey, "key_s", 1, 1);
    registerControlVariable( &cartRHandKey, "key_d", 1, 1);
    registerControlVariable( &walkTowardKey, "key_x", 1, 1);
    registerControlVariable( &wholeBodyKey, "key_v", 1, 1);
    registerControlVariable( &bothArmsrIncKey, "key_r", 1, 1);
    registerControlVariable( &bothArmsrDecKey, "key_f", 1, 1);

    registerControlVariable( positionController.stiffness.getElementsPointer(),
                             "stiffness", 1, HAPTIC_AXES);
    registerControlVariable( positionController.damping.getElementsPointer(),
                             "damping", 1, HAPTIC_AXES);

    openCloseLHandKey = headKey = openCloseRHandKey = cartLHandKey = 0;
    cartRHandKey = walkToKey = walkTowardKey = wholeBodyKey = 0;
    bothArmsrDecKey = bothArmsrIncKey = 0;

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

    // The program can still run if connection is unsucccessful
    connectToMATLAB();

    enableWandIfDisabled();
    fbMode = newFbModeAtmc = NO_FEEDBACK;

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
    const float armAngles[NUM_OF_ARM_ANGLES*2] = {
        1.38503, -0.0193124, -1.54517, -1.37153, 0.0280995, 0,
        1.38503, 0.0193124, 1.54517, 1.37153, -0.0280997, 0};
    std::vector<float>targetArmAngles;
    targetArmAngles.assign(armAngles,armAngles+NUM_OF_ARM_ANGLES*2);
    motionProxy->angleInterpolationWithSpeed( AL::ALValue::array("LArm","RArm"),
                                              targetArmAngles,
                                              0.3f);

    initTfRArm = AL::Math::Transform(
                motionProxy->getTransform("RArm", FRAME_TORSO, false));
    initTfLArm = AL::Math::Transform(
                motionProxy->getTransform("LArm", FRAME_TORSO, false));

    // Let arms stay at the last position when walking
    motionProxy->setMoveArmsEnabled(false,false);
    curMode = STOP;

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
                static unsigned moveRobotFreqCounter = 0;
                if(++moveRobotFreqCounter * INPUT_CHECK_PERIOD_SEC ==
                   MOVE_ROBOT_PERIOD_SEC)
                {
                    if(curMode != STOP)
                        moveRobot();
                    moveRobotFreqCounter = 0;
                }
            }
            commDataMutex.unlock();
            commCondVar.notify_one();
            operatorInputTimeCounter = 0;
        }
        else{
            //couldn't lock, try again next loop
            --operatorInputTimeCounter;
        }
    }

    // The NO_FEEDBACK mode will be activated after the feedback is imposed
    // so, there is no point to assign mode to NO_FEEDBACK from outside
    if(fbMode == NO_FEEDBACK)
        fbMode = newFbModeAtmc;

    static unsigned fbTickCounter = 0;
    switch (fbMode) {

    case PUSH_TO_INIT_POS_START:
        setPoint.v_lim_mult = 25.0; // decrease power
        enableWandIfDisabled();
        if(isManipulationMode(curMode))
            wd = 0,HAPTIC_START_YPOS,0,0,0;
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;
        wd_inter = w;
        fbMode = PUSH_TO_INIT_POS_CONTINUE;
    case PUSH_TO_INIT_POS_CONTINUE:
        if(++fbTickCounter > timeMsToTicks(FEEDBACK_TIME_LIMIT_MS)){
            fbMode = WAIT_AFTER_FEEDBACK;
            wd = wd_inter = w;
        }
        break;

    case TILT_START:
        enableWandIfDisabled();
        wd = w_temp = wd_inter = w;
        fbMode = TILT_CONTINUE;
    case TILT_CONTINUE:
        wd(3) = w_temp(3) + 0.004 * cos(13.5*M_PI*elapsedTime()) +
                0.02 * sin(18*M_PI*elapsedTime());
        if(++fbTickCounter > timeMsToTicks(FEEDBACK_TIME_LIMIT_MS)){
            fbMode = WAIT_AFTER_FEEDBACK;
            wd = wd_inter = w;
        }
        break;

    case WAIT_AFTER_FEEDBACK:
        if(++fbTickCounter > timeMsToTicks(FEEDBACK_WAIT_TIME_LIMIT_MS)){
            fbTickCounter = 0;
            setPoint.v_lim_mult = 75.0; // increase power again
            fbMode = newFbModeAtmc = NO_FEEDBACK;
            wd = wd_inter = w; // do it like this, just once
        }
    case NO_FEEDBACK:
        // I need to detech push and release
        buttonState = !hapticWand.readDigital(HAPTIC_BUTTON_PIN);
        if(!buttonState)
            enableWandIfDisabled();
        else
            disableWandIfEnabled();
        break;

    default:
        break;
    }

    if(hapticWandEnabled){
        // Desired position.
        wd_inter = setPoint.find_wd(wd, wd_inter, period() );
        ColumnVector<5> F = positionController.force( w, wd_inter );
        hapticWand.generateForces( period(), jointAngles, F.getElementsPointer());
    }

    return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::stop()
{
    enableWandIfDisabled();

    //Go to start position of haptic wand
    wd = 0,0,0,0,0;
    wd.setElement(2, HAPTIC_START_YPOS);
    std::cout << "going to start ypos" << std::endl;
    hapticGoToPosBlocking(0.05);
    wd = 0,0.124,0.03,0,0;
    std::cout << "going to init pos" << std::endl;
    hapticGoToPosBlocking(0.02);
    std::cout << "now disabling wand" << std::endl;

    disableWandIfEnabled();

    awarenessProxy->startAwareness();
    motionProxy->wbEnable(false);
    motionProxy->rest();

    if(matlabTCPSocket != -1)
        disconnectFromMATLAB();

    hapticWand.closeDevice();


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

    return 0;
}

void HapticTeleop::checkInputAndSetMode()
{

    // don't let it to switch whole body while not stopped
    if((curMode == STOP) && wholeBodyKey){
        commQueue.push([this](){
            useWholeBody=!useWholeBody;
            if(useWholeBody){
                motionProxy->wbEnable(true);
                motionProxy->wbFootState("Plane", "Legs"); // can be also Plane Legs
                motionProxy->post.wbEnableBalanceConstraint(false, "Legs");
            }
            else{
                motionProxy->post.wbEnable(false);
            }
        });
        std::cout << "switchWholeBody" << std::endl;
    }

    if(openCloseRHandKey){
        commQueue.push([this](){
            if(rHandOpen)
                motionProxy->post.openHand("RHand");
            else
                motionProxy->post.closeHand("RHand");
            rHandOpen = !rHandOpen;
        });
        std::cout << "openCloseRHand" << std::endl;
    }

    if(openCloseLHandKey){
        commQueue.push([this](){
            if(lHandOpen)
                motionProxy->post.openHand("LHand");
            else
                motionProxy->post.closeHand("LHand");
            lHandOpen = !lHandOpen;
        });
        std::cout << "openCloseLHand" << std::endl;
    }

    if(bothArmsrIncKey)
        bothArmsDiameter += 0.025;
    else if(bothArmsrDecKey)
        bothArmsDiameter -= 0.025;

    // I don't know why is this needed, maybe cuz of QT
    openCloseRHandKey= openCloseLHandKey= bothArmsrIncKey= bothArmsrDecKey=0;

    // mode change
    TeleopMode newMode = STOP;
    if(headKey) newMode = HEAD;
    else if(walkToKey) newMode = WALK_TO;
    else if(walkTowardKey) newMode = WALK_TOWARD;
    else if(cartLHandKey && cartRHandKey) newMode = BOTH_ARMS;
    else if(cartLHandKey) newMode = LARM;
    else if(cartRHandKey) newMode = RARM;
    // save the current w, when we switch to same mode back,
    // we will use it

    // mode switching can only be done while button is released
    if(!buttonState && (curMode != newMode)){
        if(isManipulationMode(curMode))
            lastSamples[curMode] = w;
        if(isManipulationMode(newMode))
            wd = lastSamples[newMode];
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;

        wd_inter = w;

        curMode = newMode;
        //newFbModeAtmc = NO_FEEDBACK;
        std::cout << "Mode switched to " << curMode << std::endl;
        motionProxy->stopMove();
        if(curMode == WALK_TO)
            commQueue.push([this](){
                initRobotPosWalk = motionProxy->getRobotPosition(false);
            });

    }
}

void HapticTeleop::moveRobot()
{
    // Op X is Nao Y, Op Y is Nao X
    static std::vector<float> opCoords(HAPTIC_AXES);
    opCoords[0] = ROUND(((isManipulationMode(curMode) ?
            HAPTIC_START_YPOS : HAPTIC_MID_YPOS)-w(2))
            * mappingCoefs[curMode][1] + mappingBias[curMode][1]);
    opCoords[1] = ROUND(w(1) * mappingCoefs[curMode][0] +
            mappingBias[curMode][0]);
    opCoords[2] = ROUND(w(3) * mappingCoefs[curMode][2] +
            mappingBias[curMode][2]);
    opCoords[3] = ROUND(w(5) * mappingCoefs[curMode][4] +
            mappingBias[curMode][4]);
    opCoords[4] = ROUND(w(4) * mappingCoefs[curMode][3]+
            mappingBias[curMode][3]);

    std::cout << "opCoords:" << opCoords << std::endl;

    commQueue.push([this](){
        //std::cout << "opCoords:" << opCoords << std::endl;
        using namespace AL;

        if (curMode == BOTH_ARMS){
            if(matlabTCPSocket != -1){
                std::vector<float> armAngles(2*NUM_OF_ARM_ANGLES);

                opCoords[4] = bothArmsDiameter;
                // send opCoords to matlab
                write(matlabTCPSocket,(char*)opCoords.data(),
                      opCoords.size()*sizeof(float));
                // receive target angles and feedback mode from matlab
                read(matlabTCPSocket,(char*)armAngles.data(),
                        armAngles.size()*sizeof(float));
                float fb;
                read(matlabTCPSocket,(char*)&fb,sizeof(float));
                newFbModeAtmc = (FeedbackMode)fb;

                // move arms of nao
                motionProxy->setAngles(ALValue::array("LArm","RArm"),
                                       armAngles,0.8f);
            }
            else{
                std::cout << "Warning: BOTH ARMS mode can be only used when "
                          << "connected to MATLAB script.\n";
            }
        }
        else if(curMode == RARM){
            Math::Transform targetTfRArm = initTfRArm *
                    Math::Transform(opCoords[0],opCoords[1],opCoords[2]) *
                    Math::Transform::fromRotX(opCoords[3]);
            motionProxy->transformInterpolations("RArm", FRAME_TORSO,
                    targetTfRArm.toVector(),CONTROL_AXES, 0.75f);
        }
        else if(curMode == LARM){
            Math::Transform targetTfLArm = initTfLArm *
                    Math::Transform(opCoords[0],opCoords[1],opCoords[2]) *
                    Math::Transform::fromRotX(opCoords[3]);
            motionProxy->transformInterpolations("LArm", FRAME_TORSO,
                    targetTfLArm.toVector(),CONTROL_AXES, 0.75f);
        }
        else if(curMode == HEAD){
            ALValue effectorNames = ALValue::array("HeadPitch", "HeadYaw");

            // The variables define head angles
            ALValue targetHeadAngles = ALValue::array(
                        opCoords[4],  // Pitch
                        opCoords[3]); // Yaw (haptic roll)

            motionProxy->angleInterpolationWithSpeed(effectorNames,
                                                     targetHeadAngles,0.4f);
        }
        else if(curMode == WALK_TO){
            std::vector<float> curRobotPosWalk =
                    motionProxy->getRobotPosition(false);
            if(initRobotPosWalk[0] + opCoords[0] - curRobotPosWalk[0] > 0 &&
               areOfAnyFeetBumpersPressed())
            {
                motionProxy->stopMove();
                newFbModeAtmc = PUSH_TO_INIT_POS_START;
            }
            else{
                // doesn't work well...
                motionProxy->moveTo(
                    initRobotPosWalk[0] + opCoords[0] - curRobotPosWalk[0],
                    initRobotPosWalk[1] + opCoords[1] - curRobotPosWalk[1],
                    initRobotPosWalk[2]);
                //newFbModeAtmc = NO_FEEDBACK;
            }
        }
        else if(curMode == WALK_TOWARD){
            if(opCoords[0] > 0.2 && areOfAnyFeetBumpersPressed()){
                motionProxy->stopMove();
                newFbModeAtmc = PUSH_TO_INIT_POS_START; // override
            }
            else{
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
            }

        }
    });

}

inline void HapticTeleop::enableWandIfDisabled()
{
    if(!hapticWandEnabled){
        hapticWand.enableWand();
        hapticWandEnabled = true;
    }
}

inline void HapticTeleop::disableWandIfEnabled()
{
    if(hapticWandEnabled){
        hapticWand.disableWand();
        hapticWandEnabled = false;
    }
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
    if(connect(matlabTCPSocket,(struct sockaddr *)&serv_addr,
               sizeof(serv_addr)) == -1)
    {
        std::cerr << "Couldn't connect to matlab\n";
        close(matlabTCPSocket);
        matlabTCPSocket = -1;
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
