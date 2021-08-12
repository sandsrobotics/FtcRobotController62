package org.firstinspires.ftc.teamcode.robot2020;

import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * runs the launcher's wheel, gate, arm, frog legs, and intake
 */
@Config
public class Launcher {
    ///////////////////
    // other variables//
    ///////////////////
    // servo and motor
    double spinMultiplier;

    // calibration
    AllCalibrationDataPoints calibrationData;

    // other
    boolean shutdownWheel = true;
    boolean gateOpen = false;
    boolean runWheelForward = false;
    float intakeMotorPower = 0;
    /**
     * -1 = stowed, 0 = rest, 1 = ready, 2 = down
     */
    int frogLegPos = 0;
    int lastfrogLegPos = 0;
    double targetWheelRpm;
    Robot robot;
    LauncherSettings launcherSettings;

    /**
     * constructor that uses defualt LaucherSettings
     * 
     * @param robot passed in to allow Movement class to interface and use other
     *              parts of the robot
     */
    Launcher(Robot robot) {
        launcherSettings = new LauncherSettings();
        this.robot = robot;
        initData();
    }

    /**
     * constructor that uses passed in custom LauncherSettings
     * 
     * @param robot            passed in to allow Movement class to interface and
     *                         use other parts of the robot
     * @param launcherSettings the custom settings that the launcher uses
     */
    Launcher(Robot robot, LauncherSettings launcherSettings) {
        this.launcherSettings = launcherSettings;
        this.robot = robot;
        initData();
    }

    /**
     * sets some basic values and gets the calibration data from a file
     */
    void initData() {
        spinMultiplier = 60 / launcherSettings.ticksPerRev * launcherSettings.gearRatio;
        targetWheelRpm = launcherSettings.startRPM;
        calibrationData = AllCalibrationDataPoints.setCalibrationDataPoints(AppUtil.getDefContext(),
                launcherSettings.calibrationFileName);
    }

    ///////////////////////////
    // launcher opmode control//
    ///////////////////////////
    /**
     * does the telemetry output for launcher
     */
    void telemetryDataOut() {
        if (shutdownWheel)
            robot.addTelemetry("Mode: ", "Run on trigger");
        else
            robot.addTelemetry("Mode: ", "Run using RPM");
        robot.addTelemetry("RPM", getPRM());
        robot.addTelemetry("Set RPM", targetWheelRpm);
    }

    /**
     * sets the servos that controls the launch arm and gate from a gamepad and
     * LauncherSettings
     * 
     * @param gamepad the gamepad you want to get input from
     */
    void setLauncherServos(Gamepad gamepad) {

        if (launcherSettings.launchButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime))
            autoLaunch();
        else if (launcherSettings.launchButton.getButtonReleased(gamepad))
            moveLaunchServo();

        if (launcherSettings.gateButton.getButtonPressed(gamepad)) {
            if (gateOpen) {
                closeGateServo();
            } else {
                openGateServo(0);
            }
        }
    }

    /**
     * sets the motor that controls the launch wheel from a gamepad and
     * LauncherSettings
     * 
     * @param gamepad the gamepad you want to get input from
     */
    void setLauncherWheelMotor(Gamepad gamepad) {
        // inputs
        if (launcherSettings.revDecreaseButton.getButtonPressed(gamepad)
                || launcherSettings.revDecreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime)) {
            targetWheelRpm -= launcherSettings.RPMIncrements;
            if (targetWheelRpm < 0)
                targetWheelRpm = 0;
        }

        if (launcherSettings.revIncreaseButton.getButtonPressed(gamepad)
                || launcherSettings.revIncreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime)) {
            targetWheelRpm += launcherSettings.RPMIncrements;
            if (targetWheelRpm > launcherSettings.maxRPM)
                targetWheelRpm = launcherSettings.maxRPM;
        }

        if (launcherSettings.revModeButton.getButtonPressed(gamepad)) {
            shutdownWheel = !shutdownWheel;
            if (shutdownWheel)
                closeGateServo();
        }

        // setting motor
        if (shutdownWheel)
            robot.robotHardware.launcherWheelMotor.setPower(0);// launcherSettings.revPowerSlide.getSliderValue(gamepad));
        else
            setRPM();
    }

    /**
     * sets the intake and froglegs from controls in launcherSettings
     * 
     * @param gamepad the gamepad you want to get the controls from
     */
    void setLauncherIntakeMotor(Gamepad gamepad) {
        intakeMotorPower = 0;

        if (launcherSettings.intakeInButton.getButtonPressed(gamepad))
            runWheelForward = !runWheelForward;

        else if (launcherSettings.intakeOutSlider.getButtonHeld(gamepad)) {
            runWheelForward = false;
            intakeMotorPower = -launcherSettings.intakeOutSlider.getSliderValue(gamepad);
        } else if (runWheelForward) {
            if (frogLegPos == -1)
                unstowFrogLegs(true);
            intakeMotorPower = 1;
        }

        robot.robotHardware.launcherIntakeMotor.setPower(intakeMotorPower);
    }

    /**
     * sets the froglegs from controls in launcherSettings
     * 
     * @param gamepad the gamepad you want to get the controls from
     */
    void setFrogLegServos(Gamepad gamepad) {
        if (launcherSettings.frogLegStowButton.getButtonPressed(gamepad)) {
            if (frogLegPos != 0)
                frogLegPos = 0;
            else
                frogLegPos = 2;
            setFrogLegPos(false);
        } else if (launcherSettings.frogLegToggleButton.getButtonPressed(gamepad)) {
            if (frogLegPos != 2)
                frogLegPos = 2;
            else
                frogLegPos = 1;
            setFrogLegPos(false);
        }
    }

    /**
     * runs all the launcher and intake parts plus outputs telemetry from the
     * telemetry flag
     * 
     * @param gamepad   the gamepad that controls all the launcher and intake parts
     * @param telemetry the flag that controls launcher telemetry output
     */
    void runForTeleOp(Gamepad gamepad, boolean telemetry) {
        setLauncherServos(gamepad);
        setLauncherWheelMotor(gamepad);
        setLauncherIntakeMotor(gamepad);
        setFrogLegServos(gamepad);
        if (telemetry)
            telemetryDataOut();
    }

    /////////////////////////////////////
    // auto launcher control - main goal//
    /////////////////////////////////////
    /**
     * gets launcher ready for launching, turns towrds goal and moves behind launch
     * line(if neccesary), sets RPM from table, shoots 3 disks, and gets launcher
     * ready for intake - requires RPM table, position tracking, and drive
     */
    void autonomousLaunchDisk() {
        double RPM = calibrationData.getTargetRPMAtDistance(getDistanceToGoal(true), 3);
        if (RPM == -1)
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "unable to get RPM");
        else if (robot.movement == null)
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if (!robot.robotUsage.positionUsage.positionTrackingEnabled())
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else {
            setRPM(RPM);
            goToShootingPos();
            for (int i = 0; i < 4; i++) {
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
        }
        shutdownWheel = true;
    }

    /**
     * gets launcher ready for launching, moves to launch position, sets RPM, shoots
     * 3 disks, and gets launcher ready for intake - requires position tracking, and
     * drive
     */
    void autoLaunchDiskFromLine() {
        if (!robot.robotUsage.useDrive)
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if (!robot.robotUsage.positionUsage.positionTrackingEnabled())
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else {
            openGateServoNoDelay();
            setRPM(launcherSettings.autoLaunchRPM);
            goToLine();

            for (int i = 0; i < 4; i++) {
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
            intakeMotorPower = 1;
            shutdownWheel = true;
            closeGateServo();
        }

    }

    /**
     * turns towrds goal and moves behind launch line(if neccesary) - requires
     * position tracking and drive
     */
    void goToShootingPos() {
        if (!robot.robotUsage.useDrive)
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if (!robot.robotUsage.positionUsage.positionTrackingEnabled())
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else {
            if (robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance)
                robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0,
                        launcherSettings.minLaunchDistance, getAngleToPointToPosition()),
                        robot.movement.movementSettings.finalPosSettings);
            else
                robot.movement.moveToPosition(
                        robot.positionTracker.getPositionWithOffset(0, 0, getAngleToPointToPosition()),
                        robot.movement.movementSettings.finalPosSettings);
        }
    }

    /**
     * stows frog legs and moves to the launch position - requires drive and
     * position tracking
     */
    void goToLine() {
        if (frogLegPos != -1) {
            stowFrogLegs(false);
        }
        robot.movement.moveToPosition(launcherSettings.autoLaunchPos, robot.movement.movementSettings.finalPosSettings);
    }

    //////////////////////////////////////
    // auto launcher control - power shot//
    //////////////////////////////////////
    /**
     * things to run at the start of power shot - opens gate servo, stows frog legs,
     * and sets and runs RPM for power shot
     */
    void powerShotStart() {
        openGateServoNoDelay();
        if (frogLegPos != -1) {
            stowFrogLegs(false);
        }
        setRPM(launcherSettings.powerShotRPM);
    }

    /**
     * things to run at the end of power shot - sets RPM for main goal, shuts down
     * the wheel, and closes gate servo
     */
    void powerShotEnd() {
        targetWheelRpm = launcherSettings.autoLaunchRPM;
        shutdownWheel = true;
        closeGateServo();
    }

    /**
     * automaticlly shoot the 3 power shot goals - runs start sequence, moves and
     * shoots 3 powershot goals, runs end sequence - requires position tracking and
     * drive
     * 
     * @param positions the power shot launch positions
     */
    void autoLaunchPowerShots(Position[] positions) {
        if (!robot.robotUsage.useDrive)
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if (!robot.robotUsage.positionUsage.positionTrackingEnabled())
            robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else {
            powerShotStart();
            for (int i = 0; i < 3; i++) {
                robot.movement.moveToPosition(positions[i], robot.movement.movementSettings.finalPosSettings);
                waitForRPMInTolerance(1000);
                autoLaunch();
            }
        }
        powerShotEnd();
    }

    ////////////////
    // calculations//
    ////////////////
    /**
     * gets the angle to point to goal from current position(or any position behind
     * the line if flag is enabled)
     * 
     * @param xPos            the x position of goal
     * @param yPos            the y position of goal
     * @param angleOffset     angle offset
     * @param useMinLaunchDis flag to see if robot sould be behind min launch line
     * @return the angle needed to point to goal
     */
    double getAngleToPointToPosition(double xPos, double yPos, double angleOffset, boolean useMinLaunchDis) {
        if (robot.robotUsage.positionUsage.positionTrackingEnabled()) {
            double XDiff = xPos - robot.positionTracker.currentPosition.X;
            double YDiff;
            if (useMinLaunchDis && robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance)
                YDiff = -launcherSettings.minLaunchDistance;
            else
                YDiff = yPos - robot.positionTracker.currentPosition.Y;

            return robot.scaleAngle(Math.toDegrees(Math.atan(XDiff / YDiff)) + angleOffset);
        }
        return 0;
    }

    /**
     * gets the angle to point to 0,0(origin) from current position(or position
     * behind launch line)
     * 
     * @return the angle needed to point to origin
     */
    double getAngleToPointToPosition() {
        return getAngleToPointToPosition(0, 0, 0, true);
    }

    /**
     * tells whether or not the rpm is withing a certin passed in tolerance from a
     * passed in target RPM
     * 
     * @param targetWheelRpm the target RPM for the wheel
     * @param RPMTolerance   the maximum tolerance for RPM to be in range
     * @return whether the RPM is in tolerance
     */
    boolean isRPMInTolerance(double targetWheelRpm, double RPMTolerance) {
        return Math.abs(getPRM() - targetWheelRpm) <= RPMTolerance;
    }

    /**
     * tells whether or not the rpm is within a set tolerance from a set RPM
     * 
     * @return whether the RPM is in tolerance
     */
    boolean isRPMInTolerance() {
        return isRPMInTolerance(targetWheelRpm, launcherSettings.RPMTolerance);
    }

    /**
     * gets the current RPM of the wheel
     * 
     * @return the RPM of the wheel
     */
    double getPRM() {
        return robot.robotHardware.launcherWheelMotor.getVelocity() * spinMultiplier;
    }

    /**
     * gets the distance from current position(or position behing launch line if
     * flag is enabled) to origin(0,0)
     * 
     * @param useMinLaunchDistance flag to use the minimum launch distance
     * @return the distance to origin(goal)
     */
    double getDistanceToGoal(boolean useMinLaunchDistance) {
        if (robot.robotUsage.positionUsage.positionTrackingEnabled()) {
            if (robot.positionTracker.currentPosition.Y > launcherSettings.minLaunchDistance || !useMinLaunchDistance)
                return Math.sqrt(Math.pow(robot.positionTracker.currentPosition.X, 2)
                        + Math.pow(robot.positionTracker.currentPosition.Y, 2));
            return Math.sqrt(Math.pow(robot.positionTracker.currentPosition.X, 2)
                    + Math.pow(launcherSettings.minLaunchDistance, 2));
        }
        if (robot.robotSettings.debug_methods)
            robot.addTelemetry("error in Launcher.getDistanceToGoal: ",
                    "robot cannot find distance because it does not know its positionTracker");
        return -1;
    }

    /////////
    // other//
    /////////
    /**
     * sets the target and actual RPM from paramiter RPM
     * 
     * @param RPM the RPM you want to set the wheel to
     */
    void setRPM(double RPM) {
        targetWheelRpm = RPM;
        robot.robotHardware.launcherWheelMotor.setVelocity(RPM / spinMultiplier);
    }

    /**
     * sets the RPM of the wheel from target RPM
     */
    void setRPM() {
        setRPM(targetWheelRpm);
    }

    /**
     * waits for the RPM to be in tolerance
     * 
     * @param maxMs          the longest wait time before braking(in ms)
     * @param targetWheelRpm the target RPM
     * @param RPMTolerance   the max tolerance
     */
    void waitForRPMInTolerance(long maxMs, double targetWheelRpm, double RPMTolerance) {
        long startTime = System.currentTimeMillis();
        while (!robot.stop() && System.currentTimeMillis() - startTime < maxMs) {
            if (isRPMInTolerance(targetWheelRpm, RPMTolerance))
                break;
        }
    }

    /**
     * waits for the RPm to be in tolerance from target RPM
     * 
     * @param maxMs the longest wait time before braking(in ms)
     */
    void waitForRPMInTolerance(long maxMs) {
        waitForRPMInTolerance(maxMs, targetWheelRpm, launcherSettings.RPMTolerance);
    }

    /**
     * moves the servo that controls the launcher arm
     * 
     * @param actuatorTime the time it takes the servo to move
     */
    void moveLaunchServo(long actuatorTime) {
        if (frogLegPos == 0)
            setFrogLegPos(2, false);
        robot.robotHardware.launcherLaunchServo.setPosition(launcherSettings.launcherServoLaunchAngle);
        robot.delay(actuatorTime);
        robot.robotHardware.launcherLaunchServo.setPosition(launcherSettings.launcherServoRestAngle);
    }

    /**
     * moves the servo to open and close the launcher gate
     * 
     * @param actuatorTime the time it takes for the gate to open
     */
    void openGateServo(long actuatorTime) {
        gateOpen = true;
        robot.robotHardware.launcherGateServo.setPosition(launcherSettings.gateServoLaunchAngle);
        robot.delay(actuatorTime);
    }

    /**
     * moves the servo to open the launcher gate - sets actuator time from
     * launcherSettings
     */
    void openGateServo() {
        openGateServo(launcherSettings.gateServoMoveTime);
    }

    /**
     * moves the servo to open the launcher gate without delay
     */
    void openGateServoNoDelay() {
        openGateServo(0);
    }

    /**
     * moves the servo to close the gate
     */
    void closeGateServo() {
        robot.robotHardware.launcherGateServo.setPosition(launcherSettings.gateServoRestAngle);
        gateOpen = false;
    }

    /**
     * moves the servo to open and close the launcher gate - actuator time from
     * launcherSettings
     */
    void moveLaunchServo() {
        moveLaunchServo(launcherSettings.launcherServoMoveTime);
    }

    /**
     * automatically opens the gate, shoots a disk, and moves launcher arm back
     */
    void autoLaunch() {
        if (isRPMInTolerance()) {
            if (!gateOpen)
                openGateServo();
            moveLaunchServo();
            robot.delay(launcherSettings.launcherServoMoveTime);
        }
    }

    /////////////
    // frog legs//
    /////////////
    /**
     * sets the frog legs to a certin position
     * 
     * @param pos          the poisition of the frog legs (-1 = stowed, 0 = rest, 1
     *                     = ready, 2 = down)
     * @param delayBetween whether or not there sould be a delay between opening the
     *                     2 legs
     */
    void setFrogLegPos(int pos, boolean delayBetween) {
        if (pos >= 0 && lastfrogLegPos != pos) {
            if (frogLegPos < 0)
                unstowFrogLegs(true);
            robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.FrogLegPos[pos][0]);
            if (delayBetween)
                robot.delay(launcherSettings.delayBetweenFrogLegs);
            robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.FrogLegPos[pos][1]);
            frogLegPos = pos;
            lastfrogLegPos = pos;
        }
    }

    /**
     * sets the frog legs to the value in frogLegPos
     * 
     * @param delayBetween whether or not there sould be a delay between opening the
     *                     2 legs
     */
    void setFrogLegPos(boolean delayBetween) {
        setFrogLegPos(frogLegPos, delayBetween);
    }

    /**
     * opens the frog legs to position 0
     */
    void initFrogLegs() {
        if (robot.robotUsage.useGrabber)
            robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, true);
        robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.FrogLegPos[0][0]);
        robot.delay(launcherSettings.delayBetweenFrogLegs);
        robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.FrogLegPos[0][1]);
        robot.delay(launcherSettings.timeToOpenFrogLegs);
        if (robot.robotUsage.useGrabber)
            robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.horizontalPos, false);
        frogLegPos = 0;
    }

    /**
     * opens the frog legs to position 0
     * 
     * @param useDelay whether the method should wait for the frog legs
     */
    void unstowFrogLegs(boolean useDelay) {
        if (robot.robotUsage.useGrabber && robot.robotHardware.grabberLifterMotor
                .getCurrentPosition() < robot.grabber.grabberSettings.straitUpPos)
            robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, true);
        robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.stowPos[0]);
        robot.delay(launcherSettings.delayBetweenFrogLegs);
        robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.stowPos[1]);
        if (useDelay)
            robot.delay(launcherSettings.timeToOpenFrogLegs);
        frogLegPos = 0;
    }

    /**
     * closes the frog legs
     * 
     * @param useDelay whether the method should wait for the frog legs
     */
    void stowFrogLegs(boolean useDelay) {
        if (robot.robotUsage.useGrabber && robot.robotHardware.grabberLifterMotor
                .getCurrentPosition() < robot.grabber.grabberSettings.straitUpPos)
            robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, true);
        robot.robotHardware.launcherFrogArmLeft.setPosition(launcherSettings.stowPos[0]);
        robot.delay(launcherSettings.delayBetweenFrogLegs);
        robot.robotHardware.launcherFrogArmRight.setPosition(launcherSettings.stowPos[1]);
        if (useDelay)
            robot.delay(launcherSettings.timeToCloseFrogLegs);
        frogLegPos = -1;
    }
}// class end

/**
 * the settings for the launcher
 */
class LauncherSettings {
    //////////////////
    // user variables//
    //////////////////
    // controls
    // rev
    GamepadButtonManager revIncreaseButton = new GamepadButtonManager(GamepadButtons.B);
    GamepadButtonManager revDecreaseButton = new GamepadButtonManager(GamepadButtons.X);
    // GamepadButtonManager revPowerSlide = new
    // GamepadButtonManager(GamepadButtons.leftTRIGGER);
    GamepadButtonManager revModeButton = new GamepadButtonManager(GamepadButtons.Y);
    // launch
    GamepadButtonManager launchButton = new GamepadButtonManager(GamepadButtons.A);
    GamepadButtonManager gateButton = new GamepadButtonManager(GamepadButtons.dpadDOWN);
    // intake
    GamepadButtonManager intakeInButton = new GamepadButtonManager(GamepadButtons.rightBUMPER);
    GamepadButtonManager intakeOutSlider = new GamepadButtonManager(GamepadButtons.rightTRIGGER);
    // frog Leg
    GamepadButtonManager frogLegStowButton = new GamepadButtonManager(GamepadButtons.B);
    GamepadButtonManager frogLegToggleButton = new GamepadButtonManager(GamepadButtons.X);

    int buttonHoldTime = 500;

    // motor config
    double ticksPerRev = 28;
    double gearRatio = 1;
    double maxRPM = 6000;

    // launcher servo
    double launcherServoRestAngle = 0;
    double launcherServoLaunchAngle = 0.8;
    int launcherServoMoveTime = 200;

    // gate servo
    double gateServoRestAngle = 0;
    double gateServoLaunchAngle = 1;
    int gateServoMoveTime = 325;

    // calibration data
    protected String calibrationFileName = "LauncherConfig.json";

    // base data
    double minLaunchDistance = -64; // this is how far the robot has to be from goal to launch - IN INCHES!!!

    // auto launch
    Position autoLaunchPos = new Position(17, minLaunchDistance, -12); // this is how far the robot has to be from goal
                                                                       // to launch - IN INCHES!!!
    double autoLaunchRPM = 3600; // RPM to launch from line

    // power shots
    double powerShotRPM = 3250; // RPM to launch from power shots
    Position[] powerShotPos = { new Position(18, minLaunchDistance, 0), new Position(25.5, minLaunchDistance, 0),
            new Position(33, minLaunchDistance, 0) };

    // power shot v2
    Position[] powerShotPosV2 = { new Position(24, minLaunchDistance, -7), new Position(24, minLaunchDistance, 0.5),
            new Position(24, minLaunchDistance, 7) };

    // frog legs
    double[][] FrogLegPos = { { .5, .5 }, { .7, .7 }, { .9, .9 } };
    double[] stowPos = { 0, 0 };
    int delayBetweenFrogLegs = 30;
    int timeToOpenFrogLegs = 200;
    int timeToCloseFrogLegs = 100;

    // other
    double startRPM = autoLaunchRPM;
    double RPMIncrements = 0;
    double RPMTolerance = 150;

    LauncherSettings() {
    }
}

/**
 * a clsss that stores all the data points for RPMs
 */
class AllCalibrationDataPoints {
    CalibrationDataPoint[] calibrationDataPoints;

    public CalibrationDataPoint[] getCalibrationDataPoints() {
        return calibrationDataPoints;
    }

    /**
     * sets the calibration data points
     * 
     * @param calibrationDataPoints the data points
     */
    public void setCalibrationDataPoints(CalibrationDataPoint[] calibrationDataPoints) {
        this.calibrationDataPoints = calibrationDataPoints;
    }

    public static AllCalibrationDataPoints setCalibrationDataPoints(Context context, String fileName) {
        Gson gson = new Gson();
        return gson.fromJson(FileManager.readFromFile(fileName, context), AllCalibrationDataPoints.class);
    }

    public int getTargetRPMAtDistance(double distance, int goalNum) {
        int row = -1;
        goalNum--;
        if (calibrationDataPoints != null && goalNum >= 0 && goalNum <= 2) {
            for (int i = 0; i < calibrationDataPoints.length; i++) {
                if (calibrationDataPoints[i].distance == distance)
                    return calibrationDataPoints[i].goalRPMS[goalNum];
                else if (calibrationDataPoints[i].distance > distance) {
                    row = i;
                    break;
                }
            }

            double RPMPerInch;
            double b;
            CalibrationDataPoint cdp;
            CalibrationDataPoint cdpPrevious;

            if (row == -1) {
                cdp = calibrationDataPoints[calibrationDataPoints.length - 1];
                cdpPrevious = calibrationDataPoints[calibrationDataPoints.length - 2];
            } else {
                cdp = calibrationDataPoints[row];

                if (row == 0) {
                    cdpPrevious = calibrationDataPoints[row + 1];
                } else {
                    cdpPrevious = calibrationDataPoints[row - 1];
                }
            }
            RPMPerInch = (cdp.goalRPMS[goalNum] - cdpPrevious.goalRPMS[goalNum])
                    / (cdp.distance - cdpPrevious.distance);
            b = cdp.goalRPMS[goalNum] - (cdp.distance * RPMPerInch);
            return (int) ((RPMPerInch * distance) + b);
        }
        return -1;
    }

    /**
     * a class that stores the RPMs for the goals at a certin distance
     */
    class CalibrationDataPoint {
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