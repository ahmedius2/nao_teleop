# -*- encoding: UTF-8 -*-

''' Whole Body Motion: Multiple Effectors control '''

import argparse
import motion
import almath
import time
import getch
from naoqi import ALProxy

def main(robotIP, PORT=9559):
    '''
        Example of a whole body multiple effectors control "LArm", "RArm" and "Torso"
        Warning: Needs a PoseInit before executing
                 Whole body balancer must be inactivated at the end of the script
    '''

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # end initialize proxy, begin go to Stand Init

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # end go to Stand Init, begin initialize whole body

    # Enable Whole Body Motion
    motionProxy.wbEnable(True)

    # Legs are constrained fixed
    motionProxy.wbFootState("Plane", "Legs")

    # Do not constraint Balance Motion
    # I will constrain it with haptic teleoperation
    motionProxy.wbEnableBalanceConstraint(False, "Legs")
    # end initialize whole body, define arms motions

    # Arms motion
    effectorList = ["RArm", "LArm"]
    useSensorValues = False
    frame        = motion.FRAME_ROBOT
    timesList = [ [0.8], [0.8]] # move in 0.8 seconds
    axisMaskList = [almath.AXIS_MASK_VEL, # for "LArm"
                    almath.AXIS_MASK_VEL] # for "RArm"

    print "Starting teleop\n"
    teleop = True
    while teleop:
        pathLArm = []
        pathRArm = []
        c = getch.getch()
        # Retrieve current transform from ALMotion.
        # Convert it to a transform matrix for ALMath.
        currentTfR = motionProxy.getTransform("RArm", frame, useSensorValues)
        currentTfL = motionProxy.getTransform("LArm", frame, useSensorValues)
        targetTfR  = almath.Transform(currentTfR)
        targetTfL  = almath.Transform(currentTfL)
        if c == 'w': # +x
            targetTfR.r1_c4 += 0.025
            targetTfL.r1_c4 += 0.025
        elif c == 's': # -x
            targetTfR.r1_c4 -= 0.025
            targetTfL.r1_c4 -= 0.025
        elif c == 'd': # y
            targetTfR.r2_c4 += 0.025
            targetTfL.r2_c4 -= 0.025
        elif c == 'a': # y
            targetTfR.r2_c4 -= 0.025
            targetTfL.r2_c4 += 0.025
        elif c == 'r': # +z
            targetTfR.r3_c4 += 0.025
            targetTfL.r3_c4 += 0.025
        elif c == 'f': # -z
            targetTfR.r3_c4 -= 0.025
            targetTfL.r3_c4 -= 0.025
        elif c == '1':
            motionProxy.wbEnable(True)
            motionProxy.wbFootState("Plane", "Legs")
            motionProxy.wbEnableBalanceConstraint(False, "Legs")
            print 'Whole body motion enabled'
        elif c== '2':
            motionProxy.wbEnable(False)
            print 'Whole body motion disabled'
        elif c == 'x':
            teleop = False
        print "Got input: " + c
        print targetTfR
        print targetTfL
        
        pathRArm.append(list(targetTfR.toVector()))
        pathLArm.append(list(targetTfL.toVector()))
        pathList = [pathRArm, pathLArm]
        motionProxy.transformInterpolations(effectorList, frame, pathList,
                                                     axisMaskList, timesList)

    # Deactivate whole body
    isEnabled    = False
    motionProxy.wbEnable(isEnabled)

    # Send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.3)

    # Go to rest position
    motionProxy.rest()

    # end script

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
