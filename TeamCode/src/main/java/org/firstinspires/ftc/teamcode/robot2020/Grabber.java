package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Grabber {

    //user variables below

    /////////////
    //variables//
    /////////////
    Robot robot;
    GrabberSettings grabberSettings;

    private int setEncoderPos;
    //protected double[] setServoPositions = new double[2];
    //protected boolean clawClosed = true;
    protected boolean motorReset = false;

    protected short intakeMode = 0; // 0 = off, 1 = in, 2 = out
    protected short lastIntakeMode = 2;
    protected long lastModeSwitchTime = System.currentTimeMillis();

    Grabber(Robot robot)
    {
        this.robot = robot;
        grabberSettings = new GrabberSettings();
    }
    Grabber(Robot robot, GrabberSettings grabberSettings)
    {
        this.robot = robot;
        this.grabberSettings = grabberSettings;
    }

    void initGrabberPos()
    {
        if(robot.robotHardware.grabberArmLimitSwitch.getState())
        {
            int pos = 0;
            while(robot.robotHardware.grabberArmLimitSwitch.getState() && !robot.stop())
            {
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

    void setFromControls(Gamepad gamepad)
    {
        //set encoder
        if(grabberSettings.captureButton.getButtonPressed(gamepad)) setEncoderSetPos(grabberSettings.capturePos);
        else if(grabberSettings.horizontalButton.getButtonPressed(gamepad)) setEncoderSetPos(grabberSettings.horizontalPos);
        else if(grabberSettings.putOverButton.getButtonPressed(gamepad)) setEncoderSetPos(grabberSettings.putOverPos);
        else if(grabberSettings.restPosButton.getButtonPressed(gamepad)) setEncoderSetPos(grabberSettings.restPos);

        if(grabberSettings.moveGrabberStick.getButtonHeld(gamepad)) {
            setEncoderSetPos(getEncoderSetPos() + (int)(grabberSettings.moveGrabberStick.getSliderValue(gamepad) * grabberSettings.stickToTicksMultiplier));
        }

        //set servo
        if(grabberSettings.grabButton.getButtonPressed(gamepad)){
            if(intakeMode == 0){
                if(lastIntakeMode == 2) setIntakeMode((short) 1);
                else setIntakeMode((short) 2);
            }
            else setIntakeMode((short) 0);
        }

        if(intakeMode == 1 && (System.currentTimeMillis() - lastModeSwitchTime > grabberSettings.maxIntakeTime || !robot.robotHardware.grabberIntakeLimitSwitch.getState())) setIntakeMode((short) 0);
        else if(intakeMode == 2 && System.currentTimeMillis() - lastModeSwitchTime > grabberSettings.outtakeTime) setIntakeMode((short) 0);
/*
        //set servo
        if(grabberSettings.grabButton.getButtonPressed(gamepad)) { clawClosed = !clawClosed; }
        if(clawClosed)
        {
            setServoPositions[0] = grabberSettings.servoGrabPositions[0];
            setServoPositions[1] = grabberSettings.servoGrabPositions[1];
        }
        else
        {
            setServoPositions[0] = grabberSettings.servoRestPositions[0];
            setServoPositions[1] = grabberSettings.servoRestPositions[1];
        }
 */
    }

    void runForTeleOp(Gamepad gamepad, boolean useTelemetry)
    {
        setFromControls(gamepad);
        moveAll();
        if(useTelemetry) teleOpTelemetry();
    }

    void teleOpTelemetry()
    {
        robot.addTelemetry("grabber pos", robot.robotHardware.grabberLifterMotor.getCurrentPosition());
    }

    void moveMotors()
    {
        robot.robotHardware.grabberLifterMotor.setTargetPosition(getEncoderSetPos());
        stopMotor();
    }
/*
    void moveServos()
    {
        for(int i = 0; i < 2; i++)
        {
            robot.robotHardware.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
    }
 */

    void moveServos()
    {
        if(intakeMode == 1) robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, grabberSettings.servoIntakeSpeed);
        else if(intakeMode == 2) robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, grabberSettings.servoOuttakeSpeed);
        else robot.robotHardware.setServosPower(robot.robotHardware.grabberServos, 0);
    }

    void moveAll()
    {
        moveMotors();
        moveServos();
    }

    void setIntakeMode(short mode){
        if(intakeMode != mode && mode >= 0 && mode <= 2){
            lastModeSwitchTime = System.currentTimeMillis();
            intakeMode = mode;
            if(intakeMode != 0) lastIntakeMode = mode;
        }
    }

    void setServoModeAndMove(short mode){
        setIntakeMode(mode);
        moveServos();
    }

    void setGrabberToPos(int pos, boolean waitForMotor)
    {
        setEncoderSetPos(pos);
        moveMotors();
        while(robot.robotHardware.grabberLifterMotor.isBusy() && !robot.stop() && waitForMotor) { if(stopMotor()) break; }
    }

    void setEncoderSetPos(int pos){
        setEncoderPos = pos;
        if (setEncoderPos > grabberSettings.maxMotorPos) setEncoderPos = grabberSettings.maxMotorPos;
        else if (setEncoderPos < grabberSettings.minMotorPos) setEncoderPos = grabberSettings.minMotorPos;
        //if(robot.launcher != null && robot.launcher.frogLegPos == 0 && setEncoderPos < grabberSettings.straitUpPos) setEncoderPos = grabberSettings.straitUpPos;
    }

    int getEncoderSetPos(){
        return setEncoderPos;
    }

    boolean stopMotor()
    {
        if(!robot.robotHardware.grabberArmLimitSwitch.getState() && getEncoderSetPos() <= 5)
        {
           // if(!motorReset)
           // {
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorReset = true;
            //}
            if(robot.robotHardware.grabberLifterMotor.getPower() != 0) robot.robotHardware.grabberLifterMotor.setPower(0);
            return true;
        }
        else {
            if (robot.robotHardware.grabberLifterMotor.getPower() != grabberSettings.motorPower) robot.robotHardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
            motorReset = false;
        }
        return false;
    }

    void setGrabberToIntake(){
        setIntakeMode((short) 1);
        moveServos();
    }

    void runGrabberIntake(int maxWaitTime){
        setIntakeMode((short) 1);
        moveServos();
        while(System.currentTimeMillis() - lastModeSwitchTime < maxWaitTime){
            if(!robot.robotHardware.grabberIntakeLimitSwitch.getState()) return;
        }
        setIntakeMode((short) 0);
        moveServos();
    }

    void runGrabberIntake(){runGrabberIntake(grabberSettings.maxIntakeTime);}

    void runGrabberOuttake(boolean useDelay, int delay){
        setIntakeMode((short) 2);
        moveServos();
        if(useDelay){
            robot.delay(delay);
            setIntakeMode((short) 0);
            moveServos();
        }
    }

    void runGrabberOuttake(boolean useDelay){
        runGrabberOuttake(useDelay, grabberSettings.outtakeTime);}

/*
    void setServosToPos(double[] servoPos, boolean waitForServos)
    {
        setServoPositions[0] = servoPos[0];
        setServoPositions[1] = servoPos[1];
        moveServos();
        if(waitForServos) robot.delay(grabberSettings.servoCloseTime);
    }
 */

    ////////
    //auto//
    ////////
    void autoDrop()
    {
        if(robot.robotUsage.useDrive && robot.robotUsage.positionUsage.positionTrackingEnabled()) {
            setGrabberToPos(grabberSettings.putOverPos, false);
            robot.movement.moveToPosition(new Position(robot.positionTracker.currentPosition.X, -123, 90), robot.movement.movementSettings.finalPosSettings);
            //setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
            robot.movement.moveToPosition((robot.positionTracker.getPositionWithOffset(0, 10, 0)), robot.movement.movementSettings.losePosSettings);
            setGrabberToPos(grabberSettings.capturePos, false);
        }
    }
}

class GrabberSettings
{
    ////////////////
    //user defined//
    ////////////////
    protected int maxMotorPos = 1450;
    protected int minMotorPos = -50;
    protected double motorPower = .5;

    //preset lifter functions
    protected int capturePos = 1400; //position of grabber arm when grabbing a wobble goal
    protected int horizontalPos = -50; //position of grabber arm when in storage
    protected int putOverPos = 950; //position of grabber arm to put the wobble goal over the wall
    protected int restPos = 1450; //position of grabber arm when at rest on the side of robot
    protected int straitUpPos = 500; //position of grabber arm when strait up from the robot

    //servo pos
    //protected double[] servoRestPositions = {.2, .6};
    //protected double[] servoGrabPositions = {.9, .1};
    //protected int servoCloseTime = 400; // time for the servos to close/open(in ms)
    protected double servoIntakeSpeed = 1;
    protected double servoOuttakeSpeed = -1;
    protected int outtakeTime = 300;
    protected int maxIntakeTime = 10000;

    //controls
    protected GamepadButtonManager moveGrabberStick = new GamepadButtonManager(null);//manual adjust of grabber
    protected double stickToTicksMultiplier = 20;
    protected GamepadButtonManager grabButton = new GamepadButtonManager(GamepadButtons.leftBUMPER);//open and close the grabber claws
    //preset positions
    protected GamepadButtonManager captureButton = new GamepadButtonManager(GamepadButtons.dpadDOWN);
    protected GamepadButtonManager horizontalButton = new GamepadButtonManager(GamepadButtons.dpadUP);
    protected GamepadButtonManager putOverButton = new GamepadButtonManager(GamepadButtons.dpadLEFT);
    protected GamepadButtonManager restPosButton = new GamepadButtonManager(GamepadButtons.dpadRIGHT);

    //homing
    int homingSpeed = 50;


    GrabberSettings(){}
}
