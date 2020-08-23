#!/usr/bin/env pybricks-micropython
# Copyright Jesper Larsson - Quick and simple robot experimentation script, for ev3 Linux brick
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Config parameters - General
movementSpeed = 500 # degrees / second
sensorMoveSpeed = 35 # degrees / second
moveBackDistanceThreshold = 20 # 0-100 percentage scale, has no unit

# Config parameters - IR Beacon
enableBeaconMoves = False
enableSensorAutoMove = False
posDeadZone = 40 # Upwards sensor deadzone, will not move when within this
negDeadZone = 40 # Downwards sensor deadzone, will not move when within this
infraredChannel = 1

# Touch sensors
enableTouchSensors = False

# Sensor auto movement
sensorAutoDeadzone = 3

# Init
print("Initializing...")
ev3 = EV3Brick()
ir = InfraredSensor(Port.S4)

moveMotorLeft = Motor(Port.D)
moveMotorRight = Motor(Port.A)

sensorMotorLeft = Motor(Port.C)
sensorMotorRight = Motor(Port.B)

if enableTouchSensors:
    touchSensorRight = TouchSensor(Port.S1)
    touchSensorLeft = TouchSensor(Port.S2)
print("Init OK")

# Draws text on the given row on robot LCD screen
def DrawText(textString, targetRow):
    ev3.screen.draw_text(0, targetRow * 20, textString)

# Moves robot in the given direction, 0 = stop, 1 = forward, 2 = backward. Optional time in milliseconds
def MoveRobot(direction, impulseTime = -1):
    if impulseTime == -1:
        # Continious movement
        if direction == 1:
            DrawText("CHARGE!", 0)
            moveMotorLeft.run(movementSpeed)
            moveMotorRight.run(movementSpeed)
        elif direction == 2:
            DrawText("RUN AWAY!", 0)
            moveMotorLeft.run(-movementSpeed)
            moveMotorRight.run(-movementSpeed)
        else:
            moveMotorLeft.stop()
            moveMotorRight.stop()
    else:
        # Movement impulse, will pause execution while performed
        if direction == 1:
            DrawText("Forward impulse " + str(impulseTime), 0)
            moveMotorLeft.run_time(movementSpeed, impulseTime)
            moveMotorRight.run_time(movementSpeed, impulseTime)
        elif direction == 2:
            DrawText("Backwards impulse " + str(impulseTime), 0)
            moveMotorLeft.run_time(-movementSpeed, impulseTime)
            moveMotorRight.run_time(-movementSpeed, impulseTime)
        else:
            moveMotorLeft.stop()
            moveMotorRight.stop()

# Moves sensor platform in the given direction, 0 = stop, 1 = up, 2 = down
def MoveSensors(direction):
    if sensorDirection == 1:
        DrawText("CANNONS UP!", 1)
        sensorMotorLeft.run(-sensorMoveSpeed)
        sensorMotorRight.run(-sensorMoveSpeed)
    elif sensorDirection == 2:
        DrawText("GET DOWN MR PRESIDENT!", 1)
        sensorMotorLeft.run(sensorMoveSpeed)
        sensorMotorRight.run(sensorMoveSpeed)
    else:
        sensorMotorLeft.stop()
        sensorMotorRight.stop()

# Robot state
hasRecentlyFallenBackIR = False
initialSensorAngleLeft = sensorMotorLeft.angle() # left sensor motor is used to measure the sensor angle

# Main loop
loopCount = 0 # we only update some (slow) logic every X logical frames
while (True):
    ev3.screen.clear()
    DrawText("~~ JL ROBOT " + str(loopCount) + " ~~", 4)

    # Outputs
    moveDirection = 0 # 0 = stop, 1 = forward, 2 = backward
    moveImpulseTime = -1 # milliseconds, -1 = continious
    sensorDirection = 0

    # Read infrared beacon
    if (enableBeaconMoves and loopCount % 100 == 0):
        beaconData = ir.beacon(infraredChannel)
        print(str(beaconData))

        # Back up a bit if controller gets close
        # NOTE: The distance takes a long time to update, so we need a hysteresis to make sure we only perform a small impulse rather than backing up for many seconds
        beaconDistance = beaconData[0] 
        if beaconDistance is not None and beaconDistance <= moveBackDistanceThreshold:
            if not hasRecentlyFallenBackIR:
                moveDirection = 2
                moveImpulseTime = 1500
                hasRecentlyFallenBackIR = True
        else:
            hasRecentlyFallenBackIR = False

        # Move sensor platform towards beacon platform
        beaconAngle = beaconData[1] 
        if (beaconAngle is not None and moveDirection == 0 and enableSensorAutoMove):
            if (beaconAngle >= posDeadZone):
                sensorDirection = 2
            elif (beaconAngle <= negDeadZone):
                sensorDirection = 1

    # Directional buttons on wireless controller is used for general movement
    # Left stick = move robot, right stick = move sensor platform
    for btn in ir.buttons(infraredChannel):
        if btn == Button.LEFT_UP:
            moveDirection = 1
        elif btn == Button.LEFT_DOWN:
            moveDirection = 2
        elif btn == Button.RIGHT_UP:
            sensorDirection = 1
        elif btn == Button.RIGHT_DOWN:
            sensorDirection = 2

    # Reset sensors position when not pressing them
    if (sensorDirection == 0):
        currentAngle = sensorMotorLeft.angle()
        diff = currentAngle - initialSensorAngleLeft

        if diff >= sensorAutoDeadzone:
            sensorDirection = 1
        elif  diff <= (-sensorAutoDeadzone):
            sensorDirection = 2

    # Check hit detectors (touch sensors) - Running into walls etc
    if (enableTouchSensors and loopCount % 75 == 0):
        if touchSensorRight.pressed():
            DrawText("I hit something (right side)", 2)
            if moveDirection == 1:
                moveDirection = 0
        if touchSensorLeft.pressed():
            DrawText("I hit something (left side)", 2)
            if moveDirection == 1:
                moveDirection = 0

    # Update movement motors
    MoveRobot(moveDirection, moveImpulseTime)    
    
    # Update sensor platform movement motors
    MoveSensors(sensorDirection)

    # Done
    wait(1)
    loopCount += 1
