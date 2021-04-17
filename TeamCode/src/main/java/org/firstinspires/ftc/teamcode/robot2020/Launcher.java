package org.firstinspires.ftc.teamcode.robot2020;

import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Config
public class Launcher {
    ///////////////////
    //other variables//
    ///////////////////
    //servo and motor
    double spinMultiplier;

    //calibration
    AllCalibrationDataPoints calibrationData;

    //other
    boolean shutdownWheel = true;
    boolean gateOpen = false;
    float intakeMotorPower = 0;
    int frogLegPos = 0; //0 = rest, 1 = ready, 2 = down
    int lastfrogLegPos = 0;
    double targetWheelRpm;
    Robot robot;
    LauncherSettings launcherSettings;

    Launcher(Robot robot)
    {
        launcherSettings = new LauncherSettings();
        this.robot = robot;
        initData();
    }

    Launcher(Robot robot, LauncherSettings launcherSettings)
    {
        this.launcherSettings = launcherSettings;
        this.robot = robot;
        initData();
    }

    void initData()
    {
        spinMultiplier = 60 / launcherSettings.ticksPerRev * launcherSettings.gearRatio;
        targetWheelRpm = launcherSettings.startRPM;
        calibrationData = AllCalibrationDataPoints.setCalibrationDataPoints(AppUtil.getDefContext(), launcherSettings.calibrationFileName);
    }


    ///////////////////////////
    //launcher opmode control//
    ///////////////////////////
    void telemetryDataOut()
    {
        if(shutdownWheel) robot.addTelemetry("Mode: ", "Run on trigger");
        else robot.addTelemetry("Mode: ", "Run using RPM");
        robot.addTelemetry("RPM", getPRM());
        robot.addTelemetry("Set RPM", targetWheelRpm);
    }

    void setLauncherServos(Gamepad gamepad)
    {

        if(launcherSettings.launchButton.getButtonHeld(gamepad,launcherSettings.buttonHoldTime)) autoLaunch();
        else if(launcherSettings.launchButton.getButtonReleased(gamepad)) moveLaunchServo();

        if(launcherSettings.gateButton.getButtonPressed(gamepad))
        {
            if(gateOpen) { closeGateServo(); }
            else { openGateServo(0); }
        }
    }

    void setLauncherWheelMotor(Gamepad gamepad)
    {
        //inputs
        if(launcherSettings.revDecreaseButton.getButtonPressed(gamepad) || launcherSettings.revDecreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime))
        {
            targetWheelRpm -= launcherSettings.RPMIncrements;
            if(targetWheelRpm < 0) targetWheelRpm = 0;
        }

        if(launcherSettings.revIncreaseButton.getButtonPressed(gamepad) || launcherSettings.revIncreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime))
        {
            targetWheelRpm += launcherSettings.RPMIncrements;
            if(targetWheelRpm > launcherSettings.maxRPM) targetWheelRpm = launcherSettings.maxRPM;
        }

        if(launcherSettings.revModeButton.getButtonPressed(gamepad)) {
            shutdownWheel =!shutdownWheel;
            if(shutdownWheel) closeGateServo();
        }

        //setting motor
        if (shutdownWheel) robot.robotHardware.launcherWheelMotor.setPower(0);//launcherSettings.revPowerSlide.getSliderValue(gamepad));
        else setRPM();
    }

    void setLauncherIntakeMotor(Gamepad gamepad)
    {
        if(launcherSettings.intakeInButton.getButtonPressed(gamepad)){
            if(intakeMotorPower == 1) intakeMotorPower = 0;
            else intakeMotorPower = 1;
        }
        else if(launcherSettings.intakeOutSlider.getButtonHeld(gamepad)){
            intakeMotorPower = -launcherSettings.intakeOutSlider.getSliderValue(gamepad);
        }
        else if(launcherSettings.intakeOutSlider.getButtonReleased(gamepad)) intakeMotorPower = 0;

        robot.robotHardware.launcherIntakeMotor.setPower(intakeMotorPower);
    }

    void setFrogLegServos(Gamepad gamepad){
        if(launcherSettings.frogLegStowButton.getButtonPressed(gamepad)) {
            if(frogLegPos != 0) frogLegPos = 0;
            else frogLegPos = 2;

            if(lastfrogLegPos != frogLegPos) setFrogLegPos(false);
        }
        else if(launcherSettings.frogLegToggleButton.getButtonPressed(gamepad)) {
            if(frogLegPos != 2) frogLegPos = 2;
            else frogLegPos = 1;

            if(lastfrogLegPos != frogLegPos) setFrogLegPos(false);
        }
    }

    void runForTeleOp(Gamepad gamepad, boolean telemetry)
    {
        setLauncherServos(gamepad);
        setLauncherWheelMotor(gamepad);
        setLauncherIntakeMotor(gamepad);
        setFrogLegServos(gamepad);
        if(telemetry)telemetryDataOut();
    }

    /////////////////////////////////////
    //auto launcher control - main goal//
    /////////////////////////////////////
    void autonomousLaunchDisk()
    {
        double RPM = calibrationData.getTargetRPMAtDistance(getDistanceToGoal(true), 3);
        if(RPM == -1) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "unable to get RPM");
        else if(robot.movement == null) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if(!robot.robotUsage.positionUsage.positionTrackingEnabled()) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else
        {
            setRPM(RPM);
            goToShootingPos();
            for(int i = 0; i < 4; i++) {
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
        }
        shutdownWheel = true;
    }

    void autoLaunchDiskFromLine()
    {
        if(!robot.robotUsage.useDrive) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if(!robot.robotUsage.positionUsage.positionTrackingEnabled()) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else
        {
            openGateServoNoDelay();
            setRPM(launcherSettings.autoLaunchRPM);
            goToLine();

            for(int i = 0; i < 4; i++) {
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
            intakeMotorPower = 1;
            shutdownWheel = true;
            closeGateServo();
        }

    }

    void goToShootingPos()
    {
        if(!robot.robotUsage.useDrive) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if(!robot.robotUsage.positionUsage.positionTrackingEnabled()) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else
        {
            if (robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance) robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0,  launcherSettings.minLaunchDistance, getAngleToPointToPosition()), robot.movement.movementSettings.finalPosSettings);
            else robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0, 0, getAngleToPointToPosition()), robot.movement.movementSettings.finalPosSettings);
        }
    }

    void goToLine() { robot.movement.moveToPosition(launcherSettings.autoLaunchPos, robot.movement.movementSettings.finalPosSettings); }

    //////////////////////////////////////
    //auto launcher control - power shot//
    //////////////////////////////////////
    void powerShotStart(){
        openGateServoNoDelay();
        setRPM(launcherSettings.powerShotRPM);
    }

    void powerShotEnd(){
        targetWheelRpm = launcherSettings.autoLaunchRPM;
        shutdownWheel = true;
        closeGateServo();
    }

    void autoLaunchPowerShots(Position[] positions)
    {
        if(!robot.robotUsage.useDrive) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if(!robot.robotUsage.positionUsage.positionTrackingEnabled()) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else
        {
            powerShotStart();
            for(int i = 0; i < 3; i++) {
                robot.movement.moveToPosition(positions[i], robot.movement.movementSettings.finalPosSettings);
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
        }
        powerShotEnd();
    }

    ////////////////
    //calculations//
    ////////////////
    double getAngleToPointToPosition(double xPos, double yPos, double angleOffset, boolean useMinLaunchDis)
    {
        if(robot.robotUsage.positionUsage.positionTrackingEnabled())
        {
            double XDiff = xPos - robot.positionTracker.currentPosition.X;
            double YDiff;
            if (useMinLaunchDis && robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance)
                YDiff = -launcherSettings.minLaunchDistance;
            else YDiff = yPos - robot.positionTracker.currentPosition.Y;

            return robot.scaleAngle(Math.toDegrees(Math.atan(XDiff / YDiff)) + angleOffset);
        }
        return 0;
    }

    double getAngleToPointToPosition()
    {
        return getAngleToPointToPosition(0,0,0, true);
    }

    boolean isRPMInTolerance(double targetWheelRpm, double RPMTolerance) { return Math.abs(getPRM() - targetWheelRpm) <= RPMTolerance; }

    boolean isRPMInTolerance() { return isRPMInTolerance(targetWheelRpm, launcherSettings.RPMTolerance); }

    double getPRM() { return  robot.robotHardware.launcherWheelMotor.getVelocity() * spinMultiplier; }

    double getDistanceToGoal(boolean useMinLaunchDistance)
    {
        if(robot.robotUsage.positionUsage.positionTrackingEnabled())
        {
            if (robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance || !useMinLaunchDistance)
                return Math.sqrt(Math.pow(robot.positionTracker.currentPosition.X, 2) + Math.pow(robot.positionTracker.currentPosition.Y, 2));
            return Math.sqrt(Math.pow(robot.positionTracker.currentPosition.X, 2) + Math.pow(launcherSettings.minLaunchDistance, 2));
        }
        if(robot.robotSettings.debug_methods) robot.addTelemetry("error in Launcher.getDistanceToGoal: ", "robot cannot find distance because it does not know its positionTracker");
        return -1;
    }

    /////////
    //other//
    /////////
    void setRPM(double RPM)
    {
        targetWheelRpm = RPM;
        robot.robotHardware.launcherWheelMotor.setVelocity(RPM / spinMultiplier);
    }

    void setRPM()
    {
        setRPM(targetWheelRpm);
    }

    void waitForRPMInTolerance(long maxMs, double targetWheelRpm, double RPMTolerance)
    {
        long startTime = System.currentTimeMillis();
        while(!robot.stop() && System.currentTimeMillis() - startTime < maxMs)
        {
            if(isRPMInTolerance(targetWheelRpm, RPMTolerance)) break;
        }
    }

    void waitForRPMInTolerance(long maxMs)
    {
        waitForRPMInTolerance(maxMs, targetWheelRpm, launcherSettings.RPMTolerance);
    }

    void moveLaunchServo(long actuatorTime)
    {
        robot.robotHardware.launcherLaunchServo.setPosition(launcherSettings.launcherServoLaunchAngle);
        robot.delay(actuatorTime);
        robot.robotHardware.launcherLaunchServo.setPosition(launcherSettings.launcherServoRestAngle);
    }

    void openGateServo(long actuatorTime)
    {
        gateOpen = true;
        robot.robotHardware.launcherGateServo.setPosition(launcherSettings.gateServoLaunchAngle);
        robot.delay(actuatorTime);
    }

    void openGateServo(){openGateServo(launcherSettings.gateServoMoveTime);}
    void openGateServoNoDelay(){openGateServo(0);}

    void closeGateServo()
    {
        robot.robotHardware.launcherGateServo.setPosition(launcherSettings.gateServoRestAngle);
        gateOpen = false;
    }

    void moveLaunchServo()
    {
        moveLaunchServo(launcherSettings.launcherServoMoveTime);
    }

    void autoLaunch()
    {
        if(isRPMInTolerance())
        {
            if(!gateOpen) openGateServo();
            moveLaunchServo();
            robot.delay(launcherSettings.launcherServoMoveTime);
        }
    }

    void setFrogLegPos(int pos, boolean delayBetween){
        if(pos == 0 && robot.grabber != null && robot.grabber.setEncoderPos < robot.grabber.grabberSettings.straitUpPos){
            robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, false);
        }
        robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.FrogLegPos[pos][0]);
        if(delayBetween) robot.delay(launcherSettings.delayBetweenFrogLegs);
        robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.FrogLegPos[pos][1]);
        lastfrogLegPos = pos;
    }

    void setFrogLegPos(boolean delayBetween){
        setFrogLegPos(frogLegPos, delayBetween);
    }

    void initFrogLegs(){
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, true);
        robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.FrogLegPos[0][0]);
        robot.delay(launcherSettings.delayBetweenFrogLegs);
        robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.FrogLegPos[0][1]);
    }


}// class end

class LauncherSettings
{
    //////////////////
    //user variables//
    //////////////////
    //controls
    //rev
    GamepadButtonManager revIncreaseButton = new GamepadButtonManager(GamepadButtons.B);
    GamepadButtonManager revDecreaseButton = new GamepadButtonManager(GamepadButtons.X);
    //GamepadButtonManager revPowerSlide = new GamepadButtonManager(GamepadButtons.leftTRIGGER);
    GamepadButtonManager revModeButton = new GamepadButtonManager(GamepadButtons.Y);
    //launch
    GamepadButtonManager launchButton = new GamepadButtonManager(GamepadButtons.A);
    GamepadButtonManager gateButton = new GamepadButtonManager(GamepadButtons.dpadDOWN);
    //intake
    GamepadButtonManager intakeInButton = new GamepadButtonManager(GamepadButtons.rightBUMPER);
    GamepadButtonManager intakeOutSlider = new GamepadButtonManager(GamepadButtons.rightTRIGGER);
    //frog Leg
    GamepadButtonManager frogLegStowButton = new GamepadButtonManager(GamepadButtons.B);
    GamepadButtonManager frogLegToggleButton = new GamepadButtonManager(GamepadButtons.X);

    int buttonHoldTime = 500;

    //motor config
    double ticksPerRev = 28;
    double gearRatio = 1;
    double maxRPM = 6000;

    //launcher servo
    double launcherServoRestAngle = 0;
    double launcherServoLaunchAngle = 1;
    int launcherServoMoveTime = 250;

    //gate servo
    double gateServoRestAngle = 0;
    double gateServoLaunchAngle = 0.8;
    int gateServoMoveTime = 350;

    //calibration data
    protected String calibrationFileName =  "LauncherConfig.json";

    //base data
    double minLaunchDistance = -64; //this is how far the robot has to be from goal to launch - IN INCHES!!!

    //auto launch
    Position autoLaunchPos = new Position(17, minLaunchDistance, -12); //this is how far the robot has to be from goal to launch - IN INCHES!!!
    double autoLaunchRPM = 3600; //RPM to launch from line

    //power shots
    double powerShotRPM = 3250; //RPM to launch from power shots
    Position[] powerShotPos = {
        new Position(18, minLaunchDistance, 0),
        new Position(25.5, minLaunchDistance, 0),
        new Position(33, minLaunchDistance, 0)
    };

    //power shot v2
    Position[] powerShotPosV2 = {
        new Position(24, minLaunchDistance, -7),
        new Position(24, minLaunchDistance, 0.5),
        new Position(24, minLaunchDistance, 7)
    };

    //frog legs
    double[][] FrogLegPos = {
    {.5,.5},
    {.7,.7},
    {.9,.9}};
    int delayBetweenFrogLegs = 30;

    //other
    double startRPM = autoLaunchRPM;
    double RPMIncrements = 0;
    double RPMTolerance = 150;

    LauncherSettings(){}
}



class AllCalibrationDataPoints
{
    CalibrationDataPoint[] calibrationDataPoints;

    public CalibrationDataPoint[] getCalibrationDataPoints() {
        return calibrationDataPoints;
    }

    public void setCalibrationDataPoints(CalibrationDataPoint[] calibrationDataPoints) {
        this.calibrationDataPoints = calibrationDataPoints;
    }

    public static AllCalibrationDataPoints setCalibrationDataPoints(Context context, String fileName)
    {
        Gson gson = new Gson();
        return gson.fromJson(FileManager.readFromFile(fileName, context), AllCalibrationDataPoints.class);
    }

    public int getTargetRPMAtDistance(double distance, int goalNum)
    {
        int row = -1;
        goalNum--;
        if (calibrationDataPoints != null && goalNum >= 0 && goalNum <=  2)
        {
            for(int i = 0; i < calibrationDataPoints.length; i++)
            {
                if(calibrationDataPoints[i].distance == distance) return calibrationDataPoints[i].goalRPMS[goalNum];
                else if(calibrationDataPoints[i].distance > distance)
                {
                    row = i;
                    break;
                }
            }

            double RPMPerInch;
            double b;
            CalibrationDataPoint cdp;
            CalibrationDataPoint cdpPrevious;

            if(row == -1) {
                cdp = calibrationDataPoints[calibrationDataPoints.length - 1];
                cdpPrevious = calibrationDataPoints[calibrationDataPoints.length - 2];
            }
            else {
                 cdp = calibrationDataPoints[row];

                if (row == 0) {
                    cdpPrevious = calibrationDataPoints[row + 1];
                } else {
                    cdpPrevious = calibrationDataPoints[row - 1];
                }
            }
            RPMPerInch = (cdp.goalRPMS[goalNum] - cdpPrevious.goalRPMS[goalNum]) / (cdp.distance - cdpPrevious.distance);
            b = cdp.goalRPMS[goalNum] - (cdp.distance*RPMPerInch);
            return (int)((RPMPerInch * distance) + b);
        }
        return -1;
    }

    class CalibrationDataPoint
    {
        double distance;
        int[] goalRPMS;

        public double getDistance() {
            return distance;
        }

        public void setDistance(double distance) {
            this.distance = distance;
        }

        public int[] getGoalRPMS() {
            return goalRPMS;
        }

        public void setGoalRPMS(int[] goalRPMS) {
            this.goalRPMS = goalRPMS;
        }
    }
}