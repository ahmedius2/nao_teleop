
#include "HapticTeleop.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

/**
 * This function is called when the control program is loaded to zenom.
 * Use this function to register control parameters, to register log variables
 * and to initialize control parameters.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::initialize()
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
    connect(matlabTCPSocket,(struct sockaddr *)&serv_addr,sizeof(serv_addr));

    // connect to nao
    broker = AL::ALBroker::createBroker("MyBroker", "", 0, NAO_IP_ADDR, 9559);

    motionProxy = boost::shared_ptr<AL::ALMotionProxy>(
                new AL::ALMotionProxy(broker)
                );

    awarenessProxy =  boost::shared_ptr<AL::ALBasicAwarenessProxy>(
                new AL::ALBasicAwarenessProxy(broker)
                );

    memoryProxy =  boost::shared_ptr<AL::ALMemoryProxy>(
                new AL::ALMemoryProxy(broker)
                );

    registerLogVariable( w.getElementsPointer(), "w", 1, HAPTIC_AXES );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, HAPTIC_AXES );
    registerLogVariable( wd_inter.getElementsPointer(), "wd_inter",
                         1, HAPTIC_AXES );

    registerControlVariable( &setPoint.v_lim_mult, "v_lim_mult",1,1);
    registerControlVariable( &openCloseLHandKey, "key_q", 1, 1);
    registerControlVariable( &headKey, "key_w", 1, 1);
    registerControlVariable( &openCloseRHandKey, "key_e", 1, 1);
    registerControlVariable( &cartLHandKey, "key_a", 1, 1);
    registerControlVariable( &walkToKey, "key_s", 1, 1);
    registerControlVariable( &cartRHandKey, "key_d", 1, 1);
    registerControlVariable( &walkTowardKey, "key_x", 1, 1);
    registerControlVariable( &wholeBodyKey, "key_f", 1, 1);
    registerControlVariable( positionController.stiffness.getElementsPointer(),
                             "stiffness", 1, HAPTIC_AXES);
    registerControlVariable( positionController.damping.getElementsPointer(),
                             "damping", 1, HAPTIC_AXES);

    openCloseLHandKey = headKey = openCloseRHandKey = cartLHandKey = 0;
    cartRHandKey = walkTowardKey = walkToKey = wholeBodyKey = 0;

    isButtonPushed= false;
    rHandOpen = false;
    lHandOpen = false;
    useWholeBody = false;

    hapticWand.openDevice();        // Open the q8 card
    hapticWand.calibrateWand();     // Calibrate the haptic wand

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

    if(frequency() < ALMOTION_FREQUENCY_HZ){
        std::cerr << "Your frequency must be at least 50Hz !" << std::endl;
        return -1;
    }

    hapticWand.enableWand();
    isHapticWandEnabled = true;

    w = 0,HAPTIC_START_YPOS,0,0,0;
    for(int i=0; i<MANIP_MODES; ++i)
        lastSamples[i]= w;

    wd = 0,HAPTIC_MID_YPOS,0,0,0;
    // this is stop mode start pos

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
    awarenessProxy->stopAwareness();
    // make sure joints are stiff enough to move
    motionProxy->stiffnessInterpolation(
                AL::ALValue::array("Head","LArm","RArm"), 1.0f, 0.5f);
    // Go to hand manipulation pose
    const float armAngles[] = {
        1.38503, -0.0193124, -1.54517, -1.37153, 0.0280995, 0,
        1.38503, 0.0193124, 1.54517, 1.37153, -0.0280997, 0};
    std::vector<float>targetArmAngles;
    targetArmAngles.assign(armAngles,armAngles+12);
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

    if(isButtonPushed && hapticWand.readDigital(HAPTIC_BUTTON_PIN)){
        wd = wd_inter = w;
    }

    isButtonPushed = !hapticWand.readDigital(HAPTIC_BUTTON_PIN);

    static unsigned robotMotionFreqCounter = 0;
    if(++robotMotionFreqCounter == (frequency() / ALMOTION_FREQUENCY_HZ)){
        // 50 HZ, 20ms period
        // to reach nao, we need to call methods from teleop proxy
        // but these methods involves network communication and may block caller
        // so to do this we made a queue of functions and pushed functions in it
        // and the communication thread popped and called them
        if(commDataMutex.try_lock()){
            checkKeysAndSetMode();
            moveRobot();

            commDataMutex.unlock();
            commCondVar.notify_one();
            robotMotionFreqCounter = 0;
        }
        else{
            //couldn't lock, try again next loop
            robotMotionFreqCounter -= 1;
        }
    }

    if(!isButtonPushed){
        if(!isHapticWandEnabled){
            hapticWand.enableWand();
            isHapticWandEnabled = true;
        }

        // Desired position.
        wd_inter = setPoint.find_wd(wd, wd_inter, period() );
        ColumnVector<5> F = positionController.force( w, wd_inter );
        hapticWand.generateForces( period(), jointAngles, F.getElementsPointer() );
    }
    else if(isHapticWandEnabled){
        hapticWand.disableWand();
        isHapticWandEnabled = false;
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
    if(!isHapticWandEnabled){
        hapticWand.enableWand();
        isHapticWandEnabled = true;
    }

    //Go to start position of haptic wand
    wd = 0,0,0,0,0;
    wd.setElement(2, HAPTIC_START_YPOS);
    std::cout << "going to start ypos" << std::endl;
    hapticGoToPosBlocking(0.05);
    wd = 0,0.124,0.03,0,0;
    std::cout << "going to init pos" << std::endl;
    hapticGoToPosBlocking(0.02);
    std::cout << "now disabling wand" << std::endl;

    hapticWand.disableWand();

    motionProxy->wbEnable(false);
    motionProxy->rest();
    awarenessProxy->startAwareness();

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
    hapticWand.closeDevice();

    commTask->finishTask();
    delete commTask;

    close(matlabTCPSocket);

    return 0;
}

void HapticTeleop::checkKeysAndSetMode()
{

    // don't let it to switch whole body while not stopped
    if((curMode == STOP) && wholeBodyKey){
        commQueue.push([this](){
            useWholeBody=!useWholeBody;
            if(useWholeBody){
                motionProxy->wbEnable(true);
                motionProxy->wbFootState("Plane", "Legs"); // can be also Plane Legs
                motionProxy->wbEnableBalanceConstraint(false, "Legs");
            }
            else{
                motionProxy->wbEnable(false);
            }
        });
        std::cout << "switchWholeBody" << std::endl;
    }

    if(openCloseRHandKey){
        commQueue.push([this](){
            if(rHandOpen)
                motionProxy->openHand("RHand");
            else
                motionProxy->closeHand("RHand");
            rHandOpen = !rHandOpen;
        });
        std::cout << "openCloseRHand" << std::endl;
    }

    if(openCloseLHandKey){
        commQueue.push([this](){
            if(lHandOpen)
                motionProxy->openHand("LHand");
            else
                motionProxy->closeHand("LHand");
            lHandOpen = !lHandOpen;
        });
        std::cout << "openCloseLHand" << std::endl;
    }

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
    if(!isButtonPushed && (curMode != newMode)){
        if(isManipulationMode(curMode))
            lastSamples[curMode] = w;
        if(isManipulationMode(newMode))
            wd = lastSamples[newMode];
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;

        wd_inter = w;

        curMode = newMode;

        std::cout << "Mode switched to " << curMode << std::endl;
        commQueue.push([this](){
            // make the mode switch delayed so the haptic wand can have
            // some time for returning its last position of
            // selected new mode
            // don't worry, queue will be locked by commThread while
            // this thread is sleeping so doloop cannot overflow it
            // with other calls. But the keys won't work while on mode switch.
            if(curMode == WALK_TO)
                initRobotPosWalk = motionProxy->getRobotPosition(false);
            if (curMode != STOP)
                std::this_thread::sleep_for(std::chrono::seconds(2));
        });

    }
}

void HapticTeleop::moveRobot()
{
    // Op X is Nao Y, Op Y is Nao X
    static std::vector<float> opCoords(6);
    opCoords[0] = ROUND(
                ((isManipulationMode(curMode) ?
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
    opCoords[5] = 0.0f;

    commQueue.push([this](){
        //std::cout << "opCoords:" << opCoords << std::endl;

        if (curMode == BOTH_ARMS){
            std::vector<float> armAngles = motionProxy->getAngles(
                        AL::ALValue::array(chainStr[LARM],chainStr[RARM]),false);

            // send opCoords to matlab
            write(matlabTCPSocket,(char*)opCoords.data(),
                    4*sizeof(float));
            // send angles to matlab
            write(matlabTCPSocket,(char*)armAngles.data(),
                    armAngles.size()*sizeof(float));

            // receive target angles and force vector from matlab
            read(matlabTCPSocket,(char*)armAngles.data(),
                    armAngles.size()*sizeof(float));

            // move arms of nao
            motionProxy->angleInterpolationWithSpeed(
                        AL::ALValue::array("LArm","RArm"),armAngles, 0.4f);

        }
        else if(curMode == RARM){
            AL::Math::Transform targetTfRArm = initTfRArm *
                    AL::Math::Transform(opCoords[0],opCoords[1],opCoords[2]) *
                    AL::Math::Transform::fromRotX(opCoords[3]);
            motionProxy->transformInterpolations("RArm", FRAME_TORSO,
                    targetTfRArm.toVector(),CONTROL_AXES, 0.75f);
        }
        else if(curMode == LARM){
            AL::Math::Transform targetTfLArm = initTfLArm *
                    AL::Math::Transform(opCoords[0],opCoords[1],opCoords[2]) *
                    AL::Math::Transform::fromRotX(opCoords[3]);
            motionProxy->transformInterpolations("LArm", FRAME_TORSO,
                    targetTfLArm.toVector(),CONTROL_AXES, 0.75f);
        }
        else if(curMode == HEAD){
            AL::ALValue effectorNames =
                    AL::ALValue::array("HeadPitch", "HeadYaw");

            // The variables define head angles
            AL::ALValue targetHeadAngles = AL::ALValue::array(
                        opCoords[4],  // Pitch
                        opCoords[3]); // Yaw (haptic roll)

            motionProxy->angleInterpolationWithSpeed(effectorNames,
                                                     targetHeadAngles,0.4f);
        }
        else if(curMode == WALK_TO){
            // This might not work ...
            std::vector<float> curRobotPosWalk =
                    motionProxy->getRobotPosition(false);
            motionProxy->moveTo(
                    initRobotPosWalk[0] + opCoords[0] - curRobotPosWalk[0],
                    initRobotPosWalk[1] + opCoords[1] - curRobotPosWalk[1],
                    initRobotPosWalk[2]);
        }
        else if(curMode == WALK_TOWARD){
            if((opCoords[0] >= -0.1 && opCoords[0] <= 0.1) &&
                    (opCoords[1] >= -0.1 && opCoords[1] <= 0.1))
            { // stop
                motionProxy->stopMove();
            }
            else if (opCoords[0] > 0.1 || opCoords[0] < -0.1){
                // forward or backward
                motionProxy->moveToward(opCoords[0], 0, opCoords[1]);
            }
            else if (opCoords[1] > 0.1 || opCoords[1] < -0.1){
                // walk horizontal (crab walk :))
                motionProxy->moveToward(0, opCoords[1], 0);
            }
        }


    });

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
