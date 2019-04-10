
#include "HapticTeleop.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include "NAO_jacob0.h"
//#include <eigen3/unsupported/Eigen/MatrixFunctions>

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

    automoProxy = boost::shared_ptr<AL::ALAutonomousMovesProxy>(
                new AL::ALAutonomousMovesProxy(broker));
    automoProxy->setBackgroundStrategy("none");

    //postureProxy = boost::shared_ptr<AL::ALRobotPostureProxy>(
    //            new AL::ALRobotPostureProxy(broker));

    registerLogVariable(F.getElementsPointer(), "Haptic_Force", 1, HAPTIC_AXES);
    registerLogVariable(w.getElementsPointer(), "w", 1, HAPTIC_AXES );
    registerLogVariable(opCoords, "opCoords", 1, HAPTIC_AXES );

#ifdef DEBUG
    //registerLogVariable(wd.getElementsPointer(),"wd", 1, HAPTIC_AXES );
    registerLogVariable(manipLRArmsLog, "Manipulability_measures", 1, 2);
    registerLogVariable(&fbCooldownLog, "Feedback_cooldown", 1 , 1);
    registerLogVariable(&buttonState, "Button", 1 , 1);
    registerLogVariable(&curTeleopModeLog, "Teleop_mode", 1, 1);
    registerLogVariable(&curFbModeLog, "Feedback_mode", 1, 1);
    registerLogVariable(cHeadAnglesLog, "Head_angles_cur",        1, 2);
    registerLogVariable(tLArmPosLog,"Target_left_arm_position",   1, 6);
    registerLogVariable(tRArmPosLog,"Target_right_arm_position",  1, 6);
    registerLogVariable(cLArmPosLog,"Current_left_arm_position",  1, 6);
    registerLogVariable(cRArmPosLog,"Current_right_arm_position", 1, 6);
    registerLogVariable(cLArmAnglesLog,"Current_left_arm_angles",  1, 6);
    registerLogVariable(cRArmAnglesLog,"Current_right_arm_angles", 1, 6);
    registerLogVariable(bumpersLog,"Bumpers", 1,4);
    registerLogVariable(cWorldPosLog,"World_pos", 1,3);
    registerLogVariable(cVelLog,"Velocity", 1,3);
#endif

    registerControlVariable( &modeSelectedByUser, "_selected_mode",  1, 1);
    registerControlVariable( &rHandStateFromUser, "_rightHandState", 1, 1);
    registerControlVariable( &lHandStateFromUser, "_leftHandState",  1, 1);
    registerControlVariable( &wbStateFromUser,    "_wholeBodyState", 1, 1);
    registerControlVariable( &distBetwArms,       "_distBetwArms",   1, 1);

    registerControlVariable( &armSpeed, "Arm speed", 1, 1);
    registerControlVariable( &sleepAmountMs, "Sleep amount", 1, 1);

    sleepAmountMs=200;
    armSpeed = 0.6;
    rHandStateFromUser = lHandStateFromUser = wbStateFromUser = 0;
    modeSelectedByUser = STOP;
    curTeleopMode = STOP;
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

    modeSelectedByUser = STOP;
    curTeleopMode = STOP;

    manipLRArms[1] = 0;
    manipLRArms[0] = 0;


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

    if( commDataMutex.try_lock() ){
        if(commQueue.empty()){
            checkInputAndSetMode();
            if(curTeleopMode != STOP)
                moveRobot();
        }
        commDataMutex.unlock();
        commCondVar.notify_one();
        // I need to wait for some time before locking
    }

#ifdef DEBUG
//    curTeleopModeLog = curTeleopMode;
//    curFbModeLog = curFbMode;
//    fbCooldownLog = fbCooldown;
//    std::cerr << "Teleop mode:" << curTeleopMode <<
//                 " Feedback mode:" << curFbMode << " Cooldown:" <<
//                 fbCooldown << " hapticWandForceEnabled:" <<
//                 hapticWandForceEnabled << std::endl << std::flush;
#endif

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
        enableForceIfDisabled(period()/**5*/);
        if(isArmMode(curTeleopMode))
            wd = 0,HAPTIC_START_YPOS,0,0,0;
        else
            wd = 0,HAPTIC_MID_YPOS,0,0,0;
        wd_inter = w;
        curFbMode = FEEDBACK_CNT;
        break;

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
        });
    }
}

void HapticTeleop::moveRobot()
{
    // Op X is Nao Y, Op Y is Nao X
#ifdef DEBUG
    static std::vector<float> cHeadAngles(2), cBumpers(4),
            tLArmPos(6), tRArmPos(6), cLArmPos(6), cRArmPos(6),
            cLArmAngles(6), cRArmAngles(6), cWorldPos(3), cVel(3);
#endif

    opCoords[0] = ROUND(((isArmMode(curTeleopMode) ?
            HAPTIC_START_YPOS : HAPTIC_MID_YPOS) - w(2))
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

        static auto callCounter = 0;

        if (curTeleopMode == BOTH_ARMS){
            const float xc = 0.15; // x center
            const float yc = 0;    // y center
            const float zc = 0.02; // z center

            Math::Transform targetLArmTf =
                    Math::Transform::fromPosition( xc + opCoords[0] ,
                        yc + opCoords[1] +(distBetwArms*cos(opCoords[3])) ,
                        zc + opCoords[2] -(distBetwArms*sin(opCoords[3]))) *
                    Math::Transform::fromRotX( -opCoords[3]-(M_PI/2.0));

            Math::Transform targetRArmTf =
                    Math::Transform::fromPosition( xc + opCoords[0] ,
                        yc + opCoords[1] -(distBetwArms*cos(opCoords[3])) ,
                        zc + opCoords[2] +(distBetwArms*sin(opCoords[3]))) *
                    Math::Transform::fromRotX( -opCoords[3]+(M_PI/2.0));

            auto lArmVec = targetLArmTf.toVector();
            auto rArmVec = targetRArmTf.toVector();
            ALValue names = ALValue::array("LArm","RArm");
            if(callCounter++ * sleepAmountMs >= CALL_TIME_MS){
                motionProxy->setTransforms( "LArm", FRAME_TORSO,
                        lArmVec, armSpeed, 15);
                motionProxy->setTransforms( "RArm", FRAME_TORSO,
                        rArmVec, armSpeed, 15);
                callCounter=0;
            }

            auto armAnglesVec = motionProxy->getAngles(names, false);

            Eigen::MatrixXd jacob(6,5);
            NaoRArm_jacob0(jacob, armAnglesVec.data());
            Eigen::VectorXcd eigs = (jacob.transpose() * jacob).eigenvalues();
            manipLRArms[1] =
                sqrt(eigs.real().maxCoeff()) / sqrt(eigs.real().minCoeff());

            std::cout << "Manipulability right arm:" << manipLRArms[1] <<
                         std::endl;

            if((float)manipLRArms[1] < manipThreshold*1.25){
                // With a high probability, this part is not needed,?????
                NaoLArm_jacob0(jacob,armAnglesVec.data()+6);
                eigs = (jacob.transpose() * jacob).eigenvalues();
                manipLRArms[0] = sqrt(eigs.real().maxCoeff()) /
                        sqrt(eigs.real().minCoeff());

                std::cout << "Manipulability left arm:" << manipLRArms[0] <<
                             std::endl;
            }

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
                    ((float)manipLRArms[0] > manipThreshold*1.25 ||
                     (float)manipLRArms[1] > manipThreshold*1.25))
            {
                fbStartTime = elapsedTime();
                newFbMode = FEEDBACK_START;
            }


#ifdef DEBUG
            tLArmPos =
                    Math::position6DFromTransform(targetLArmTf).toVector();
            tRArmPos =
                    Math::position6DFromTransform(targetRArmTf).toVector();
            motionProxy->waitUntilMoveIsFinished();
            cLArmPos = motionProxy->getPosition("LArm",FRAME_TORSO,false);
            cRArmPos = motionProxy->getPosition("RArm",FRAME_TORSO,false);
            std::copy(armAnglesVec.begin(),armAnglesVec.begin()+6,
                      cLArmAngles.begin());
            std::copy(armAnglesVec.begin()+6,armAnglesVec.end(),
                      cRArmAngles.begin());
#endif

        }
        else if(curTeleopMode == RARM || curTeleopMode == LARM){
            bool isRArm = (curTeleopMode == RARM);
            std::string armName = (isRArm ? "RArm" : "LArm");

            Math::Transform targetTfArm = Math::Transform::fromPosition(
                        opCoords[0],opCoords[1],opCoords[2],
                        (isRArm ? -1 : 1) * opCoords[3],0,0) *
                        (isRArm ? initTfRArm : initTfLArm);

            //calling setTransforms very fast causes not to move
            if(callCounter++ * sleepAmountMs >= CALL_TIME_MS){
                motionProxy->setTransforms(armName, FRAME_TORSO,
                        targetTfArm.toVector(), armSpeed, CONTROL_AXES);
                callCounter = 0;
            }

            auto armAnglesVec = motionProxy->getAngles(armName, false);

            Eigen::MatrixXd jacob(6,5);
            if(isRArm)
                NaoRArm_jacob0(jacob,armAnglesVec.data());
            else
                NaoLArm_jacob0(jacob,armAnglesVec.data());

            Eigen::VectorXcd eigs = (jacob.transpose() * jacob).eigenvalues();

            manipLRArms[isRArm ? 1 : 0] =
                sqrt(eigs.real().maxCoeff()) / sqrt(eigs.real().minCoeff());

            std::cout << "Manipulability max/min:" <<
                         manipLRArms[isRArm?1:0] << std::endl;

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
                    (manipLRArms[(isRArm ? 1 : 0)]) > manipThreshold)
            {
                // FEEDBACK TIME
                // Translation feedback
//                Eigen::MatrixXd jacobTra = jacob.block(0,0,3,5);
//                auto vels = (jacobTra * jacobTra.transpose()).sqrt();
//                std::cout << "X vel:" << vels.row(0).norm() << std::endl;
//                std::cout << "Y vel:" << vels.row(1).norm() << std::endl;
//                std::cout << "Z vel:" << vels.row(2).norm() << std::endl;

//                Eigen::MatrixXd jacobRot = jacob.block(3,0,,5);
//                vels = (jacobRot * jacobRot.transpose()).sqrt();
//                std::cout << "Roll vel:" << vels.row(0).norm() << std::endl;
//                std::cout << "Pith vel:" << vels.row(1).norm() << std::endl;

                fbStartTime = elapsedTime();
                newFbMode = FEEDBACK_START;
            }

#ifdef DEBUG
            (isRArm ? tRArmPos : tLArmPos) =
                Math::position6DFromTransform(targetTfArm).toVector();
            (isRArm ? cRArmPos : cLArmPos) =
                    motionProxy->getPosition(armName, FRAME_TORSO, false);
            (isRArm ? cRArmAngles : cLArmAngles ) = armAnglesVec;
#endif
        }
        else if(curTeleopMode == HEAD){
            const ALValue jointNames =
                    ALValue::array("HeadPitch", "HeadYaw");

            // The variables define head angles
            ALValue targetAngles = ALValue::array(
                        opCoords[4],  // Pitch
                        opCoords[3]); // Yaw (haptic roll)

            motionProxy->setAngles(jointNames,
                                   targetAngles,0.3f);
#ifdef DEBUG
            cHeadAngles = motionProxy->getAngles(
                        ALValue::array("HeadPitch", "HeadYaw"),false);
#endif

        }
        else if(curTeleopMode == WALK_TOWARD){ // new system???
            static std::vector<float> cBumpers(4);
#ifdef DEBUG
            cWorldPos = motionProxy->getRobotPosition(false);
            cVel = motionProxy->getRobotVelocity();
            if(areOfAnyFeetBumpersPressed(cBumpers) && opCoords[0] > 0.2){
#else
            if(opCoords[0] > 0.2 && areOfAnyFeetBumpersPressed(cBumpers)){
#endif
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
#ifdef DEBUG
    // dont forget we have the mutex right now
    if (curTeleopMode == BOTH_ARMS){
        std::copy(tLArmPos.begin(),tLArmPos.end(),tLArmPosLog);
        std::copy(tRArmPos.begin(),tRArmPos.end(),tRArmPosLog);
        std::copy(cLArmPos.begin(),cLArmPos.end(),cLArmPosLog);
        std::copy(cRArmPos.begin(),cRArmPos.end(),cRArmPosLog);
        std::copy(cLArmAngles.begin(),cLArmAngles.end(),cLArmAnglesLog);
        std::copy(cRArmAngles.begin(),cRArmAngles.end(),cRArmAnglesLog);
        std::copy(manipLRArms, manipLRArms+2, manipLRArmsLog);
    }
    else if(curTeleopMode == RARM || curTeleopMode == LARM){
        bool isRArm = (curTeleopMode == RARM);
        std::vector<float>& tArmPos = (isRArm ? tRArmPos : tLArmPos);
        std::copy(tArmPos.begin(), tArmPos.end(),
                  (isRArm ? tRArmPosLog : tLArmPosLog));

        std::vector<float>& cArmPos = (isRArm ? cRArmPos : cLArmPos);
        std::copy(cArmPos.begin(), cArmPos.end(),
                  (isRArm ? cRArmPosLog : cLArmPosLog));

        std::vector<float>& cArmAngles = (isRArm ? cRArmAngles : cLArmAngles);
        std::copy(cArmAngles.begin(), cArmAngles.end(),
                  (isRArm ? cRArmAnglesLog : cLArmAnglesLog));

        std::copy(manipLRArms, manipLRArms+2, manipLRArmsLog);
    }
    else if(curTeleopMode == HEAD){
        cHeadAnglesLog[0] = cHeadAngles[0];
        cHeadAnglesLog[1] = cHeadAngles[1];
    }
    else if (curTeleopMode == WALK_TOWARD){
        for(int i=0; i<4; ++i)
            bumpersLog[i] = (double)cBumpers[i];
        std::copy(cWorldPos.begin(), cWorldPos.end(), cWorldPosLog);
        std::copy(cVel.begin(), cVel.end(), cVelLog);
    }
#endif
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

bool HapticTeleop::areOfAnyFeetBumpersPressed(std::vector<float>& bumpersOutp)
{
    AL::ALValue keys = AL::ALValue::array(
        "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
        "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
        "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
        "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value");
    bumpersOutp = memoryProxy->getListData(keys);
    return bumpersOutp[0] || bumpersOutp[1] || bumpersOutp[2] || bumpersOutp[3];
}
