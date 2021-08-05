package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * class contains all methods and variables related to and used for movement of
 * the robot
 */
@Config
public class Movement {

    ///////////////////
    // other variables//
    ///////////////////
    // for move robot
    protected double[] lastMovePowers = new double[] { 0, 0, 0 };
    protected double speedMultiplier = 1;

    // other class
    Robot robot;
    MovementSettings movementSettings;

    /**
     * constructor for Movment class that uses default movement settings
     * 
     * @param robot passed in to allow Movement class to interface and use other
     *              parts of the robot
     */
    Movement(Robot robot) {
        movementSettings = new MovementSettings();
        this.robot = robot;
    }

    /**
     * constructor for Movment class that uses custom movement settings
     * 
     * @param robot            passed in to allow Movement class to interface and
     *                         use other parts of the robot
     * @param movementSettings custom settings for Movement class
     */
    Movement(Robot robot, MovementSettings movementSettings) {
        this.movementSettings = movementSettings;
        this.robot = robot;
    }

    ////////////////
    // turn methods//
    ////////////////

    /**
     * turns the robot to a specified angle
     * 
     * @param targetAngle                    the angle you want the robot to turn to
     * @param tolerance                      the maximun tolerance to finish the
     *                                       turn
     * @param numberOfTimesToStayInTolerance the number of times to be in tolerance
     *                                       before turn finishes
     * @param maxRuntime                     the maximum time the turn can take
     *                                       before quiting
     * @param maxSpeed                       the maximum speed of the turn
     * @param turnPID                        the pid tune for getting power from
     *                                       angle error
     */
    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime,
            double maxSpeed, PIDCoefficients turnPID) {
        double error = robot.findAngleError(robot.positionTracker.currentPosition.R, targetAngle);

        if (Math.abs(error) > tolerance) {

            int numberOfTimesInTolerance = 0;
            PID pid = new PID(turnPID, -maxSpeed, maxSpeed);

            while (numberOfTimesInTolerance < numberOfTimesToStayInTolerance && maxRuntime > 0 && !robot.stop()) {
                error = robot.findAngleError(robot.positionTracker.currentPosition.R, targetAngle);
                pid.updatePID(error);

                robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                        moveRobotPowers(0, 0, pid.returnValue(), false, true));

                if (Math.abs(error) < tolerance)
                    numberOfTimesInTolerance++;
                else
                    numberOfTimesInTolerance = 0;

                maxRuntime--;

            }
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
        }
    }

    /**
     * turns the robot to a specified angle - uses default PID in
     * movementSettings.turnPID
     * 
     * @param targetAngle                    the angle you want the robot to turn to
     * @param tolerance                      the maximun tolerance to finish the
     *                                       turn
     * @param numberOfTimesToStayInTolerance the number of times to be in tolerance
     *                                       before turn finishes
     * @param maxRuntime                     the maximum time the turn can take
     *                                       before quitting
     * @param maxSpeed                       the maximum speed of the turn
     */
    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime,
            double maxSpeed) {
        turnToAngle(targetAngle, tolerance, numberOfTimesToStayInTolerance, maxRuntime, maxSpeed,
                movementSettings.turnPID);
    }

    /**
     * turns the robot to a specified angle
     * 
     * @param targetAngle the angle you want the robot to turn to
     * @param rtas        the settings for turnToAngle
     */
    void turnToAngle(double targetAngle, RotToAngleSettings rtas) {
        if (!rtas.isPIDValid()) {
            rtas.turnPID = movementSettings.turnPID;
        }
        turnToAngle(targetAngle, rtas.tol, rtas.timesInTol, rtas.maxRuntime, rtas.maxPower, rtas.turnPID);
    }

    ////////////////
    // move methods//
    ////////////////

    /**
     * moves the robot to a specified position and angle
     * 
     * @param targetPos              the target position for the robot
     * @param tol                    the tolerance for the final position
     * @param timesToStayInTolerance the number of times to be in tolerance before
     *                               move finishes
     * @param maxLoops               the maximum number of loops before move quits
     * @param moveXPID               the x pid tune for getting power from x error
     * @param moveYPID               the y pid tune for getting power from y error
     * @param turnPID                the pid tune for getting power from angle error
     * @param maxSpeed               the maximum speed of move
     */
    void moveToPosition(Position targetPos, double[] tol, int timesToStayInTolerance, int maxLoops,
            PIDCoefficients moveXPID, PIDCoefficients moveYPID, PIDCoefficients turnPID, double maxSpeed) {
        if (robot.robotUsage.positionUsage.positionTrackingEnabled()) {
            int loops = 0;
            Position currentPos = robot.positionTracker.currentPosition;

            if (Math.abs(targetPos.X - currentPos.X) > tol[0] || Math.abs(targetPos.Y - currentPos.Y) > tol[1]
                    || Math.abs(targetPos.R - currentPos.R) > tol[2]) {
                PID xPID = new PID(moveXPID, -maxSpeed, maxSpeed);
                PID yPID = new PID(moveYPID, -maxSpeed, maxSpeed);
                PID rotPID = new PID(turnPID, -maxSpeed, maxSpeed);

                double[] powers = new double[3];

                double errorVectorRot;
                double errorVectorMag;

                int numOfTimesInTolerance = 0;

                while (!robot.stop() && (loops < maxLoops) && (numOfTimesInTolerance < timesToStayInTolerance)) {
                    currentPos = robot.positionTracker.currentPosition;

                    // calculate the error vector
                    errorVectorMag = Math.sqrt(
                            Math.pow((targetPos.X - currentPos.X), 2) + Math.pow((targetPos.Y - currentPos.Y), 2));
                    errorVectorRot = Math
                            .toDegrees(Math.atan2((targetPos.X - currentPos.X), (targetPos.Y - currentPos.Y)));

                    // take out robot rotation
                    errorVectorRot -= currentPos.R;
                    errorVectorRot = robot.scaleAngle(errorVectorRot);

                    // get the errors comps
                    powers[0] = xPID.updatePIDAndReturnValue(errorVectorMag * Math.sin(Math.toRadians(errorVectorRot)));
                    powers[1] = yPID.updatePIDAndReturnValue(errorVectorMag * Math.cos(Math.toRadians(errorVectorRot)));
                    powers[2] = rotPID.updatePIDAndReturnValue(robot.findAngleError(currentPos.R, targetPos.R));

                    if (Math.abs(targetPos.X - currentPos.X) < tol[0] && Math.abs(targetPos.Y - currentPos.Y) < tol[1]
                            && Math.abs(targetPos.R - currentPos.R) < tol[2])
                        numOfTimesInTolerance++;
                    else
                        numOfTimesInTolerance = 0;

                    robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                            moveRobotPowers(powers[0], powers[1], powers[2], false, true));
                    if (robot.robotSettings.debug_methods || robot.positionTracker.drawDashboardField) {
                        robot.startTelemetry();
                        if (robot.robotSettings.debug_methods) {
                            robot.addTelemetry("x: ", robot.positionTracker.currentPosition.X);
                            robot.addTelemetry("y: ", robot.positionTracker.currentPosition.Y);
                            robot.addTelemetry("rot: ", robot.positionTracker.currentPosition.R);
                            robot.addTelemetry("error mag: ", errorVectorMag);
                            robot.addTelemetry("error rot: ", errorVectorRot);
                        }
                        robot.positionTracker.drawAllPositions();
                        robot.sendTelemetry();
                    }
                    loops++;
                }
            }
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
        } else if (robot.robotSettings.debug_methods)
            robot.addTelemetry("error in Movement.moveToPosition: ",
                    "robot can not move to positionTracker because it does not know its positionTracker");
    }

    /**
     * moves the robot to a specified position and angle - uses default PIDs from
     * movementSettings
     * 
     * @param targetPos              the target position for the robot
     * @param tol                    the tolerance for the final position
     * @param timesToStayInTolerance the number of times to be in tolerance before
     *                               move finishes
     * @param maxLoops               the maximum number of loops before move quits
     * @param maxSpeed               the maximum speed of move
     */
    void moveToPosition(Position targetPos, double[] tol, int timesToStayInTolerance, int maxLoops, double maxSpeed) {
        moveToPosition(targetPos, tol, timesToStayInTolerance, maxLoops, movementSettings.moveXPID,
                movementSettings.moveYPID, movementSettings.turnPID, maxSpeed);
    }

    /**
     * moves the robot to a specified position and angle
     * 
     * @param targetPos the target position for the robot
     * @param mtps      the settings for move to position
     */
    void moveToPosition(Position targetPos, MoveToPositionSettings mtps) {
        if (mtps.isPIDValid())
            moveToPosition(targetPos, mtps.tol, mtps.timesInTol, mtps.maxRuntime, mtps.xPID, mtps.yPID, mtps.turnPID,
                    mtps.maxPower);
        else
            moveToPosition(targetPos, mtps.tol, mtps.timesInTol, mtps.maxRuntime, mtps.maxPower);
    }

    //////////
    // teleOp//
    //////////

    /**
     * sets the speed multiplier for moveForTeleOp
     * 
     * @param amount the speed multiplier you want to set it to
     */
    void setSpeedMultiplier(double amount) {
        if (amount > movementSettings.speedMultiplierMax) {
            if (robot.robotSettings.debug_methods)
                robot.addTelemetry("warning in Movement.setSpeedMultiplier: ",
                        "set speed is greater than max speed. setting to max speed");
            amount = movementSettings.speedMultiplierMax;
        } else if (amount < movementSettings.speedMultiplierMin) {
            if (robot.robotSettings.debug_methods)
                robot.addTelemetry("warning in Movement.setSpeedMultiplier: ",
                        "set speed is less than min speed. setting to min speed");
            amount = movementSettings.speedMultiplierMin;
        }
        speedMultiplier = amount;
    }

    /**
     * sets the speed multiplier to max from movementSettings
     */
    void setSpeedMultiplierToMax() {
        speedMultiplier = movementSettings.speedMultiplierMax;
    }

    /**
     * sets the speed multiplier to min from movementSettings
     */
    void setSpeedMultiplierToMin() {
        speedMultiplier = movementSettings.speedMultiplierMin;
    }

    /**
     * moves the robot from gamepad for teleOp
     * 
     * @param gamepad      the gamepad used for moving the robot
     * @param breakButton  button for stoping the robot immediately - if gamepad is
     *                     null it will use gamepad from above
     * @param useTelemetry turns on and off the telemetry for movement
     */
    void moveForTeleOp(Gamepad gamepad, GamepadButtonManager breakButton, boolean useTelemetry) {
        if (breakButton.gamepad == null)
            breakButton.gamepad = gamepad;
        if (breakButton.getButtonHeld()) {
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
            lastMovePowers[0] = 0;
            lastMovePowers[1] = 0;
            lastMovePowers[2] = 0;
        } else
            robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                    moveRobotPowers(movementSettings.XMoveStick.getSliderValue(gamepad),
                            -movementSettings.YMoveStick.getSliderValue(gamepad),
                            movementSettings.RotMoveStick.getSliderValue(gamepad), true, true));
        if (useTelemetry)
            teleOpTelemetry();
    }

    /**
     * moves the robot from gamepad for teleOp - without breaks
     * 
     * @param gamepad      the gamepad used for moving the robot
     * @param useTelemetry turns on and off the telemetry for movement
     */
    void moveForTeleOp(Gamepad gamepad, boolean useTelemetry) {
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                moveRobotPowers(movementSettings.XMoveStick.getSliderValue(gamepad),
                        -movementSettings.YMoveStick.getSliderValue(gamepad),
                        movementSettings.RotMoveStick.getSliderValue(gamepad), true, true));
        if (useTelemetry)
            teleOpTelemetry();
    }

    /**
     * telemetry for movement
     */
    void teleOpTelemetry() {
        robot.addTelemetry("rot", robot.positionTracker.currentPosition.R);
    }

    /**
     * moves the robot based on a feild oriented plane
     * 
     * @param gamepad the gamepad used for moving the robot
     * @param offset  the angle offset from 0 for the robot to move strait
     */
    void headlessMoveForTeleOp(Gamepad gamepad, double offset) {
        double curAngle = -robot.positionTracker.currentPosition.R + offset;
        curAngle = robot.scaleAngle(curAngle);
        double gamepadAngle = robot.getAngleFromXY(-gamepad.left_stick_x, -gamepad.left_stick_y);
        double error = -robot.findAngleError(curAngle, gamepadAngle);
        double power = Math.max(Math.abs(gamepad.left_stick_x), Math.abs(gamepad.left_stick_y));
        double[] XY = robot.getXYFromAngle(error);
        XY[0] *= power;
        XY[1] *= power;
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                moveRobotPowers(XY[0], XY[1], gamepad.right_stick_x, true, true));
    }

    /////////
    // other//
    /////////

    /**
     * moves the robot based on specific powers
     * 
     * @param X                    the x power of the robot
     * @param Y                    the y power of the robot
     * @param rotation             the rotation power of the robot
     * @param applySpeedMultiplier whether or not the speed multiplyer is apllied or
     *                             just set to 1
     * @param applyMoveSmoothing   whether or not to apply smoothing
     */
    void moveRobot(double X, double Y, double rotation, boolean applySpeedMultiplier, boolean applyMoveSmoothing) {
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors,
                moveRobotPowers(X, Y, rotation, applySpeedMultiplier, applyMoveSmoothing));
    }

    /**
     * the powers needed to move the robot at certin rates
     * 
     * @param X                    the x power of the robot
     * @param Y                    the y power of the robot
     * @param rotation             the rotation power of the robot
     * @param applySpeedMultiplier whether or not the speed multiplyer is apllied or
     *                             just set to 1
     * @param applyMoveSmoothing   whether or not to apply smoothing
     * @return the powers that the motors need to move at certin rates
     */
    double[] moveRobotPowers(double X, double Y, double rotation, boolean applySpeedMultiplier,
            boolean applyMoveSmoothing) {
        if (applyMoveSmoothing) {
            // smoothing for XYR
            X = applySmoothing(X, lastMovePowers[0], movementSettings.moveXSmoothingSteps);
            Y = applySmoothing(Y, lastMovePowers[1], movementSettings.moveYSmoothingSteps);
            rotation = applySmoothing(rotation, lastMovePowers[2], movementSettings.rotationSmoothingSteps);

            lastMovePowers[0] = X;
            lastMovePowers[1] = Y;
            lastMovePowers[2] = rotation;
        }

        double[] arr = { (Y + X + rotation), (Y - X + rotation), (Y - X - rotation), (Y + X - rotation) };
        double highestPower = 0;

        for (double val : arr)
            if (val > highestPower)
                highestPower = val;
        if (highestPower > 1)
            for (int i = 0; i < 4; i++)
                arr[i] /= highestPower;
        if (applySpeedMultiplier)
            for (int i = 0; i < 4; i++)
                arr[i] *= speedMultiplier;
        return (arr);
    }

    /**
     * gives the powers for moving the robot at a certin angle and power
     * 
     * @param angle          angle for the robot to move at
     * @param basePower      power for the robot to move at
     * @param applySmoothing whether or not to apply smoothing to powers
     * @return the powers for the motors to move at angle
     */
    double[] moveAtAnglePowers(double angle, double basePower, boolean applySmoothing) {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0] * basePower;
        double y = arr[1] * basePower;

        return moveRobotPowers(x, y, 0, false, applySmoothing);
    }

    /**
     * applies smoothing to a value
     * 
     * @param currentVal     the current set value
     * @param lastVal        the last value
     * @param smoothingSteps the step to add or subtract from last value
     * @return the smoothed value
     */
    double applySmoothing(double currentVal, double lastVal, double smoothingSteps) {
        if (currentVal - lastVal > smoothingSteps) {
            currentVal = lastVal + smoothingSteps;
        } else if (currentVal - lastVal < -smoothingSteps) {
            currentVal = lastVal - smoothingSteps;
        }
        return currentVal;
    }
}

/**
 * settings used for the Movement class
 */
@Config
class MovementSettings {
    //////////////////
    // user variables//
    //////////////////
    public PIDCoefficients turnPID = new PIDCoefficients(.03, 0, 0);
    public PIDCoefficients moveXPID = new PIDCoefficients(.07, 0, 0);
    public PIDCoefficients moveYPID = new PIDCoefficients(.07, 0, 0);
    public double moveXSmoothingSteps = 1;
    public double moveYSmoothingSteps = 1;
    public double rotationSmoothingSteps = 1;

    public double speedMultiplierMin = .2;
    public double speedMultiplierMax = 2;

    // controls
    GamepadButtonManager XMoveStick = new GamepadButtonManager(GamepadButtons.leftJoyStickX);
    GamepadButtonManager YMoveStick = new GamepadButtonManager(GamepadButtons.leftJoyStickY);
    GamepadButtonManager RotMoveStick = new GamepadButtonManager(GamepadButtons.rightJoyStickX);

    // presets
    MoveToPositionSettings finalPosSettings = new MoveToPositionSettings(new double[] { .75, .75, .5 }, 20, 10000, 1);
    MoveToPositionSettings losePosSettings = new MoveToPositionSettings(new double[] { 4, 4, 7.5 }, 1, 10000, 1);
    MoveToPositionSettings capturePosSettings = new MoveToPositionSettings(new double[] { .75, .75, .5 }, 20, 3000, 1);
    MoveToPositionSettings mediumPosSettings = new MoveToPositionSettings(new double[] { 1.5, 1.5, 1 }, 20, 10000, 1);

    /**
     * default constructor that uses preset values
     */
    MovementSettings() {
    }
}

/**
 * settings used for moveToPosition
 */
class MoveToPositionSettings {
    double[] tol;
    int timesInTol;
    int maxRuntime;
    double maxPower;
    PIDCoefficients turnPID = null;
    PIDCoefficients xPID = null;
    PIDCoefficients yPID = null;

    /**
     * constructor that keeps all the values empty
     */
    MoveToPositionSettings() {
    }

    /**
     * constructor that sets all values exept PIDs
     * 
     * @param tol        the tolerance for the final position
     * @param timesInTol the number of times to be in tolerance before move finishes
     * @param maxRuntime the maximum number of loops before move quits
     * @param maxPower   the maximum speed of move
     */
    MoveToPositionSettings(double[] tol, int timesInTol, int maxRuntime, double maxPower) {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
    }

    /**
     * constructor that sets all values
     * 
     * @param tol        the tolerance for the final position
     * @param timesInTol the number of times to be in tolerance before move finishes
     * @param maxRuntime the maximum number of loops before move quits
     * @param maxPower   the maximum speed of move
     * @param xPID       the x pid tune for getting power from x error
     * @param yPID       the y pid tune for getting power from y error
     * @param turnPID    the pid tune for getting power from angle error
     */
    MoveToPositionSettings(double[] tol, int timesInTol, int maxRuntime, double maxPower, PIDCoefficients xPID,
            PIDCoefficients yPID, PIDCoefficients turnPID) {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
        this.xPID = xPID;
        this.yPID = yPID;
        this.turnPID = turnPID;
    }

    /**
     * checks whether the PIDs are defined or not
     * @return whether or not the PIDs are valid
     */
    public boolean isPIDValid() {
        return turnPID != null && xPID != null && yPID != null;
    }

    /**
     * returns RotToAngleSettings from MoveToPositionSettings
     * @return settings for turnToAngle
     */
    RotToAngleSettings toRotAngleSettings() {
        if (turnPID != null)
            return new RotToAngleSettings(tol[2], timesInTol, maxRuntime, maxPower, turnPID);
        return new RotToAngleSettings(tol[2], timesInTol, maxRuntime, maxPower);
    }

}

class RotToAngleSettings {
    double tol;
    int timesInTol;
    int maxRuntime;
    double maxPower;
    PIDCoefficients turnPID;

    /**
     * constructor that keeps all the values empty
     */
    RotToAngleSettings() {
    }

    /**
     * constructor that sets all values exept PIDs
     * 
     * @param tol        the tolerance for the final angle
     * @param timesInTol the number of times to be in tolerance before turn finishes
     * @param maxRuntime the maximum number of loops before turn quits
     * @param maxPower   the maximum speed of turn
     */
    RotToAngleSettings(double tol, int timesInTol, int maxRuntime, double maxPower) {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
    }

    /**
     * constructor that sets all values
     * 
     * @param tol        the tolerance for the final turn
     * @param timesInTol the number of times to be in tolerance before turn finishes
     * @param maxRuntime the maximum number of loops before turn quits
     * @param maxPower   the maximum speed of turn
     * @param turnPID    the PID tune for getting power from angle error
     */
    RotToAngleSettings(double tol, int timesInTol, int maxRuntime, double maxPower, PIDCoefficients turnPID) {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
        this.turnPID = turnPID;
    }

    /**
     * checks whether the PID is defined or not
     * @return whether or not the PID is valid
     */
    boolean isPIDValid() {
        return turnPID != null;
    }
}
