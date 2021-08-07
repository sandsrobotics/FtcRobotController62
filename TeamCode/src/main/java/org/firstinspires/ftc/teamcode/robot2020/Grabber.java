package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * runs the grabber arm and intake
 */
public class Grabber {

    // user variables below

    /////////////
    // variables//
    /////////////
    Robot robot;
    GrabberSettings grabberSettings;

    private int setEncoderPos;
    // protected double[] setServoPositions = new double[2];
    // protected boolean clawClosed = true;
    protected boolean motorReset = false;

    protected short intakeMode = 0; // 0 = off, 1 = in, 2 = out
    protected short lastIntakeMode = 2;
    protected long lastModeSwitchTime = System.currentTimeMillis();

    /**
     * constructor that uses default settings
     * 
     * @param robot passed in to allow Movement class to interface and use other
     *              parts of the robot
     */
    Grabber(Robot robot) {
        this.robot = robot;
        grabberSettings = new GrabberSettings();
    }

    /**
     * constructor that uses custom settings
     * 
     * @param robot           passed in to allow Movement class to interface and use
     *                        other parts of the robot
     * @param grabberSettings the custom settings you want to use
     */
    Grabber(Robot robot, GrabberSettings grabberSettings) {
        this.robot = robot;
        this.grabberSettings = grabberSettings;
    }

    /**
     * sets the grabber to 0 using a switch and then resets the motor
     */
    void initGrabberPos() {
        if (robot.robotHardware.grabberArmLimitSwitch.getState()) {
            int pos = 0;
            while (robot.robotHardware.grabberArmLimitSwitch.getState() && !robot.stop()) {
                pos -= grabberSettings.homingSpeed;
                robot.robotHardware.grabberLifterMotor.setTargetPosition(pos);
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.robotHardware.grabberLifterMotor.setTargetPosition(0);
        robot.robotHardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * sets the grabber arm and intake from controls in GrabberSettings
     * 
     * @param gamepad the gamepad you want to get the controls from
     */
    void setFromControls(Gamepad gamepad) {
        // set encoder
        if (grabberSettings.captureButton.getButtonPressed(gamepad))
            setEncoderSetPos(grabberSettings.capturePos);
        else if (grabberSettings.horizontalButton.getButtonPressed(gamepad))
            setEncoderSetPos(grabberSettings.horizontalPos);
        else if (grabberSettings.putOverButton.getButtonPressed(gamepad))
            setEncoderSetPos(grabberSettings.putOverPos);
        else if (grabberSettings.restPosButton.getButtonPressed(gamepad))
            setEncoderSetPos(grabberSettings.restPos);

        if (grabberSettings.moveGrabberStick.getButtonHeld(gamepad)) {
            setEncoderSetPos(getEncoderSetPos() + (int) (grabberSettings.moveGrabberStick.getSliderValue(gamepad)
                    * grabberSettings.stickToTicksMultiplier));
        }

        // set servo
        if (grabberSettings.grabButton.getButtonPressed(gamepad)) {
            if (intakeMode == 0) {
                if (lastIntakeMode == 2)
                    setIntakeMode((short) 1);
                else
                    setIntakeMode((short) 2);
            } else
                setIntakeMode((short) 0);
        }

        if (intakeMode == 1 && (System.currentTimeMillis() - lastModeSwitchTime > grabberSettings.maxIntakeTime
                || !robot.robotHardware.grabberIntakeLimitSwitch.getState()))
            setIntakeMode((short) 0);
        else if (intakeMode == 2 && System.currentTimeMillis() - lastModeSwitchTime > grabberSettings.outtakeTime)
            setIntakeMode((short) 0);
        /*
         * //set servo if(grabberSettings.grabButton.getButtonPressed(gamepad)) {
         * clawClosed = !clawClosed; } if(clawClosed) { setServoPositions[0] =
         * grabberSettings.servoGrabPositions[0]; setServoPositions[1] =
         * grabberSettings.servoGrabPositions[1]; } else { setServoPositions[0] =
         * grabberSettings.servoRestPositions[0]; setServoPositions[1] =
         * grabberSettings.servoRestPositions[1]; }
         */
    }

    /**
     * sets and moves the grabber arm and intake from controls plus adds telemetry
     * based on flag
     * 
     * @param gamepad      the gamepad you want the controls from
     * @param useTelemetry flag to add telemetry
     */
    void runForTeleOp(Gamepad gamepad, boolean useTelemetry) {
        setFromControls(gamepad);
        moveAll();
        if (useTelemetry)
            teleOpTelemetry();
    }

    /**
     * adds telemetry for grabber position
     */
    void teleOpTelemetry() {
        robot.addTelemetry("grabber pos", robot.robotHardware.grabberLifterMotor.getCurrentPosition());
    }

    /**
     * moves the motor for the grabber arm
     */
    void moveMotors() {
        robot.robotHardware.grabberLifterMotor.setTargetPosition(getEncoderSetPos());
        stopMotor();
    }

    /*
     * void moveServos() { for(int i = 0; i < 2; i++) {
     * robot.robotHardware.grabberServos.get(i).setPosition(setServoPositions[i]); }
     * }
     */

    /**
     * moves the motors for the grabber intake
     */
    void moveServos() {
        if (intakeMode == 1)
            robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, grabberSettings.servoIntakeSpeed);
        else if (intakeMode == 2)
            robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, grabberSettings.servoOuttakeSpeed);
        else
            robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, 0);
    }

    /**
     * moves both the grabber intake and arm
     */
    void moveAll() {
        moveMotors();
        moveServos();
    }

    /**
     * sets the mode of the grabber intake
     * 
     * @param mode the mode of the intake - 0 = off, 1 = in, 2 = out
     */
    void setIntakeMode(short mode) {
        if (intakeMode != mode && mode >= 0 && mode <= 2) {
            lastModeSwitchTime = System.currentTimeMillis();
            intakeMode = mode;
            if (intakeMode != 0)
                lastIntakeMode = mode;
        }
    }

    /**
     * sets the mode of the grabber intake and moves it
     * 
     * @param mode the mode of the intake - 0 = off, 1 = in, 2 = out
     */
    void setServoModeAndMove(short mode) {
        setIntakeMode(mode);
        moveServos();
    }

    /**
     * sets the position and moves the grabber arm - optionally it will wait for
     * motor to get to position
     * 
     * @param pos          the position of the grabber arm
     * @param waitForMotor flag to wait for the motor to reach set position
     */
    void setGrabberToPos(int pos, boolean waitForMotor) {
        setEncoderSetPos(pos);
        moveMotors();
        while (robot.robotHardware.grabberLifterMotor.isBusy() && !robot.stop() && waitForMotor) {
            if (stopMotor())
                break;
        }
    }

    /**
     * sets the set position for the grabber arm
     * 
     * @param pos the position you want the arm to go to
     */
    void setEncoderSetPos(int pos) {
        setEncoderPos = pos;
        if (setEncoderPos > grabberSettings.maxMotorPos)
            setEncoderPos = grabberSettings.maxMotorPos;
        else if (setEncoderPos < grabberSettings.minMotorPos)
            setEncoderPos = grabberSettings.minMotorPos;
        // if(robot.launcher != null && robot.launcher.frogLegPos == 0 && setEncoderPos
        // < grabberSettings.straitUpPos) setEncoderPos = grabberSettings.straitUpPos;
    }

    /**
     * gets the set postion for encoder arm
     * 
     * @return the set postion of encoder arm
     */
    int getEncoderSetPos() {
        return setEncoderPos;
    }

    /**
     * desides whether or not to stop the motor and then stops motor - on start it
     * also resets the motor
     * 
     * @return whether the motor is stopped or not
     */
    boolean stopMotor() {
        if (!robot.robotHardware.grabberArmLimitSwitch.getState() && getEncoderSetPos() <= 5) {
            // if(!motorReset)
            // {
            robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorReset = true;
            // }
            if (robot.robotHardware.grabberLifterMotor.getPower() != 0)
                robot.robotHardware.grabberLifterMotor.setPower(0);
            return true;
        } else if (getEncoderSetPos() == grabberSettings.maxMotorPos
                && robot.robotHardware.grabberLifterMotor.getCurrentPosition() > grabberSettings.straitUpPos + 100) {
            robot.robotHardware.grabberLifterMotor.setPower(0);
            return true;
        } else {
            if (robot.robotHardware.grabberLifterMotor.getPower() != grabberSettings.motorPower)
                robot.robotHardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
            motorReset = false;
        }
        return false;
    }

    /**
     * sets the grabber intake mode to intake and then moves the motors
     */
    void setGrabberToIntake() {
        setIntakeMode((short) 1);
        moveServos();
    }

    /**
     * sets the grabber intake mode to off and then stops the motors
     */
    void setGrabberToIntakeOff() {
        setIntakeMode((short) 0);
        moveServos();
    }

    /**
     * sets the grabber intake mode to intake - moves the motor until limit switch
     * is pressed or the max wait time is reached
     * 
     * @param maxWaitTime the max time to wait for limit swith before stopping
     *                    intake
     */
    void runGrabberIntake(int maxWaitTime) {
        setIntakeMode((short) 1);
        moveServos();
        while (System.currentTimeMillis() - lastModeSwitchTime < maxWaitTime) {
            if (!robot.robotHardware.grabberIntakeLimitSwitch.getState())
                return;
        }
        setIntakeMode((short) 0);
        moveServos();
    }

    /**
     * same function as runGrabberIntake(int maxWaitTime) but uses
     * grabberSettings.maxIntakeTime instead
     */
    void runGrabberIntake() {
        runGrabberIntake(grabberSettings.maxIntakeTime);
    }

    /**
     * sets the grabber intake mode to outtake and runs until delay is over
     * 
     * @param useDelay whether or not to use a delay then turn off intake
     * @param delay    the delay time before turning off intake
     */
    void runGrabberOuttake(boolean useDelay, int delay) {
        setIntakeMode((short) 2);
        moveServos();
        if (useDelay) {
            robot.delay(delay);
            setIntakeMode((short) 0);
            moveServos();
        }
    }

    /**
     * sets the grabber intake mode to outtake and runs until delay is over - delay
     * is gotten from grabberSettings.outtakeTime
     * 
     * @param useDelay whether or not to use a delay then turn off intake
     */
    void runGrabberOuttake(boolean useDelay) {
        runGrabberOuttake(useDelay, grabberSettings.outtakeTime);
    }

    /*
     * void setServosToPos(double[] servoPos, boolean waitForServos) {
     * setServoPositions[0] = servoPos[0]; setServoPositions[1] = servoPos[1];
     * moveServos(); if(waitForServos) robot.delay(grabberSettings.servoCloseTime);
     * }
     */

    ////////
    // auto//
    ////////

    /**
     * automatically lifts grabber arm, moves to drop area, drops goal, moves back,
     * and lowers grabber arm - requres position tracking and drive
     */
    void autoDrop() {
        if (robot.robotUsage.useDrive && robot.robotUsage.positionUsage.positionTrackingEnabled()) {
            setGrabberToPos(grabberSettings.putOverPos, false);
            robot.movement.moveToPosition(new Position(robot.positionTracker.currentPosition.X, -123, 90),
                    robot.movement.movementSettings.finalPosSettings);
            // setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
            robot.movement.moveToPosition((robot.positionTracker.getPositionWithOffset(0, 10, 0)),
                    robot.movement.movementSettings.losePosSettings);
            setGrabberToPos(grabberSettings.capturePos, false);
        }
    }
}

class GrabberSettings {
    ////////////////
    // user defined//
    ////////////////
    protected int maxMotorPos = 1450;
    protected int minMotorPos = -50;
    protected double motorPower = .5;

    // preset lifter functions
    protected int capturePos = 1400; // position of grabber arm when grabbing a wobble goal
    protected int drivePos = 1200;
    protected int horizontalPos = -50; // position of grabber arm when in storage
    protected int putOverPos = 950; // position of grabber arm to put the wobble goal over the wall
    protected int restPos = 1450; // position of grabber arm when at rest on the side of robot
    protected int straitUpPos = 500; // position of grabber arm when strait up from the robot

    // servo pos
    // protected double[] servoRestPositions = {.2, .6};
    // protected double[] servoGrabPositions = {.9, .1};
    // protected int servoCloseTime = 400; // time for the servos to close/open(in
    // ms)
    protected double servoIntakeSpeed = 1;
    protected double servoOuttakeSpeed = -1;
    protected int outtakeTime = 300;
    protected int maxIntakeTime = 10000;

    // controls
    protected GamepadButtonManager moveGrabberStick = new GamepadButtonManager(null);// manual adjust of grabber
    protected double stickToTicksMultiplier = 20;
    protected GamepadButtonManager grabButton = new GamepadButtonManager(GamepadButtons.leftBUMPER);// open and close
                                                                                                    // the grabber claws
    // preset positions
    protected GamepadButtonManager captureButton = new GamepadButtonManager(GamepadButtons.dpadDOWN);
    protected GamepadButtonManager horizontalButton = new GamepadButtonManager(GamepadButtons.dpadUP);
    protected GamepadButtonManager putOverButton = new GamepadButtonManager(GamepadButtons.dpadLEFT);
    protected GamepadButtonManager restPosButton = new GamepadButtonManager(GamepadButtons.dpadRIGHT);

    // homing
    int homingSpeed = 50;

    /**
     * default constructor that uses preset values for GrabberSettings
     */
    GrabberSettings() {
    }
}
