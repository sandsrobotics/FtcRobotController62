package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Movement
{

    ///////////////////
    //other variables//
    ///////////////////
    //for move robot
    protected double[] lastMovePowers = new double[]{0,0,0};
    protected double speedMultiplier = 1;

    //other class
    Robot robot;
    MovementSettings movementSettings;

    Movement(Robot robot)
    {
        movementSettings = new MovementSettings();
        this.robot = robot;
    }
    Movement(Robot robot, MovementSettings movementSettings)
    {
        this.movementSettings = movementSettings;
        this.robot = robot;
    }


    ////////////////
    //turn methods//
    ////////////////

    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime, double maxSpeed, PIDCoefficients turnPID)
    {
        double error = robot.findAngleError(robot.positionTracker.currentPosition.R, targetAngle);

        if(Math.abs(error) > tolerance) {

            int numberOfTimesInTolerance = 0;
            PID pid = new PID(turnPID, -maxSpeed, maxSpeed);

            while (numberOfTimesInTolerance < numberOfTimesToStayInTolerance && maxRuntime > 0 && !robot.stop())
            {
                error = robot.findAngleError(robot.positionTracker.currentPosition.R, targetAngle);
                pid.updatePID(error);

                robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(0, 0, pid.returnValue(),false,true));

                if (Math.abs(error) < tolerance) numberOfTimesInTolerance++;
                else numberOfTimesInTolerance = 0;

                maxRuntime--;

            }
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
        }
    }

    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime, double maxSpeed) {
        turnToAngle(targetAngle, tolerance, numberOfTimesToStayInTolerance, maxRuntime, maxSpeed, movementSettings.turnPID);
    }

    void turnToAngle(double targetAngle, RotToAngleSettings rtas)
    {
        if(!rtas.isPIDValid()){rtas.turnPID = movementSettings.turnPID;}
        turnToAngle(targetAngle, rtas.tol, rtas.timesInTol, rtas.maxRuntime, rtas.maxPower, rtas.turnPID);
    }

    ////////////////
    //move methods//
    ////////////////
    void moveToPosition(Position targetPos, double[] tol, int timesToStayInTolerance, int maxLoops, PIDCoefficients moveXPID, PIDCoefficients moveYPID, PIDCoefficients turnPID, double maxSpeed)
    {
        if(robot.robotUsage.positionUsage.positionTrackingEnabled())
        {
            int loops = 0;
            Position currentPos = robot.positionTracker.currentPosition;

            if (Math.abs(targetPos.X - currentPos.X) > tol[0] || Math.abs(targetPos.Y - currentPos.Y) > tol[1] || Math.abs(targetPos.R - currentPos.R) > tol[2]) {
                PID xPID = new PID(moveXPID, -maxSpeed, maxSpeed);
                PID yPID = new PID(moveYPID, -maxSpeed, maxSpeed);
                PID rotPID = new PID(turnPID, -maxSpeed, maxSpeed);

                double[] powers = new double[3];

                double errorVectorRot;
                double errorVectorMag;

                int numOfTimesInTolerance = 0;

                while (!robot.stop() && (loops < maxLoops) && (numOfTimesInTolerance < timesToStayInTolerance)) {
                    currentPos = robot.positionTracker.currentPosition;

                    //calculate the error vector
                    errorVectorMag = Math.sqrt(Math.pow((targetPos.X - currentPos.X), 2) + Math.pow((targetPos.Y - currentPos.Y), 2));
                    errorVectorRot = Math.toDegrees(Math.atan2((targetPos.X - currentPos.X), (targetPos.Y - currentPos.Y)));

                    //take out robot rotation
                    errorVectorRot -= currentPos.R;
                    errorVectorRot = robot.scaleAngle(errorVectorRot);

                    //get the errors comps
                    powers[0] = xPID.updatePIDAndReturnValue(errorVectorMag * Math.sin(Math.toRadians(errorVectorRot)));
                    powers[1] = yPID.updatePIDAndReturnValue(errorVectorMag * Math.cos(Math.toRadians(errorVectorRot)));
                    powers[2] = rotPID.updatePIDAndReturnValue(robot.findAngleError(currentPos.R, targetPos.R));

                    if (Math.abs(targetPos.X - currentPos.X) < tol[0] && Math.abs(targetPos.Y - currentPos.Y) < tol[1] && Math.abs(targetPos.R - currentPos.R) < tol[2])
                        numOfTimesInTolerance++;
                    else numOfTimesInTolerance = 0;

                    robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(powers[0], powers[1], powers[2], false, true));
                    if(robot.robotSettings.debug_methods || robot.positionTracker.drawDashboardField){
                        if(robot.robotSettings.debug_methods) {
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
        }
        else if(robot.robotSettings.debug_methods) robot.addTelemetry("error in Movement.moveToPosition: ", "robot can not move to positionTracker because it does not know its positionTracker");
    }

    void moveToPosition(Position targetPos, double[] tol, int timesToStayInTolerance, int maxLoops, double maxSpeed) { moveToPosition(targetPos, tol, timesToStayInTolerance, maxLoops, movementSettings.moveXPID, movementSettings.moveYPID, movementSettings.turnPID, maxSpeed); }

    void moveToPosition(Position targetPos, MoveToPositionSettings mtps)
    {
        if(mtps.isPIDValid()) moveToPosition(targetPos, mtps.tol, mtps.timesInTol, mtps.maxRuntime, mtps.xPID, mtps.yPID, mtps.turnPID, mtps.maxPower);
        else moveToPosition(targetPos, mtps.tol, mtps.timesInTol, mtps.maxRuntime, mtps.maxPower);
    }

    //////////
    //teleOp//
    //////////

    void setSpeedMultiplier(double amount)
    {
        if(amount > movementSettings.speedMultiplierMax)
        {
            if (robot.robotSettings.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is greater than max speed. setting to max speed");
            amount = movementSettings.speedMultiplierMax;
        }
        else if(amount < movementSettings.speedMultiplierMin)
        {
            if (robot.robotSettings.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is less than min speed. setting to min speed");
            amount = movementSettings.speedMultiplierMin;
        }
        speedMultiplier = amount;
    }
    void setSpeedMultiplierToMax() { speedMultiplier = movementSettings.speedMultiplierMax; }
    void setSpeedMultiplierToMin() { speedMultiplier = movementSettings.speedMultiplierMin; }

    void moveForTeleOp(Gamepad gamepad, GamepadButtonManager breakButton, boolean useTelemetry)
    {
        if(breakButton.gamepad == null) breakButton.gamepad = gamepad;
        if(breakButton.getButtonHeld())
        {
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
            lastMovePowers[0] = 0; lastMovePowers[1] = 0; lastMovePowers[2] = 0;
        }
        else robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(movementSettings.XMoveStick.getSliderValue(gamepad), -movementSettings.YMoveStick.getSliderValue(gamepad), movementSettings.RotMoveStick.getSliderValue(gamepad), true,true));
        if(useTelemetry) teleOpTelemetry();
    }
    void moveForTeleOp(Gamepad gamepad, boolean useTelemetry)
    {
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(movementSettings.XMoveStick.getSliderValue(gamepad), -movementSettings.YMoveStick.getSliderValue(gamepad), movementSettings.RotMoveStick.getSliderValue(gamepad), true,true));
        if(useTelemetry) teleOpTelemetry();
    }

    void teleOpTelemetry()
    {
        robot.addTelemetry("rot", robot.positionTracker.currentPosition.R);
    }

    /*
    void headlessMoveForTeleOp(Gamepad gamepad1, double offset)
    {
        double curAngle = -robot.positionTracker.currentPosition.R + offset;
        curAngle = robot.scaleAngle(curAngle);
        double gamepadAngle = robot.getAngleFromXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double error = -robot.findAngleError(curAngle,gamepadAngle);
        double power = Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y));
        double[] XY = robot.getXYFromAngle(error);
        XY[0] *= power;
        XY[1] *= power;
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(XY[0],XY[1],gamepad1.right_stick_x, true, true));
    }

     */

    /////////
    //other//
    /////////

    void moveRobot(double X, double Y, double rotation, boolean applySpeedMultiplier, boolean applyMoveSmoothing)
    {
        robot.robotHardware.setMotorsToSeparatePowersArrayList(robot.robotHardware.driveMotors, moveRobotPowers(X, Y, rotation, applySpeedMultiplier, applyMoveSmoothing));
    }

    double[] moveRobotPowers(double X, double Y, double rotation, boolean applySpeedMultiplier, boolean applyMoveSmoothing)
    {
        if(applyMoveSmoothing)
        {
            //smoothing for XYR
            X = applySmoothing(X, lastMovePowers[0], movementSettings.moveXSmoothingSteps);
            Y = applySmoothing(Y, lastMovePowers[1], movementSettings.moveYSmoothingSteps);
            rotation = applySmoothing(rotation, lastMovePowers[2], movementSettings.rotationSmoothingSteps);

            lastMovePowers[0] = X;
            lastMovePowers[1] = Y;
            lastMovePowers[2] = rotation;
        }

        double[] arr =
        {
            (Y + X + rotation),
            (Y - X + rotation),
            (Y - X - rotation),
            (Y + X - rotation)
        };
        double highestPower = 0;

        for(double val:arr) if(val > highestPower) highestPower = val;
        if(highestPower > 1) for(int i = 0; i < 4; i++) arr[i] /= highestPower;
        if(applySpeedMultiplier)for(int i = 0; i < 4; i++) arr[i] *= speedMultiplier;
        return (arr);
    }

    double[] moveAtAnglePowers(double angle, double basePower, boolean applySmoothing)
    {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0] * basePower;
        double y = arr[1] * basePower;

        return moveRobotPowers(x,y,0, false,applySmoothing);
    }

    double applySmoothing(double currentVal, double lastVal, double smoothingSteps)
    {
        if(currentVal - lastVal > smoothingSteps) { currentVal = lastVal + smoothingSteps; }
        else if(currentVal - lastVal < -smoothingSteps) { currentVal = lastVal - smoothingSteps; }
        return currentVal;
    }
}

@Config
class MovementSettings
{
    //////////////////
    //user variables//
    //////////////////
    public PIDCoefficients turnPID = new PIDCoefficients(.03,0,0);
    public PIDCoefficients moveXPID = new PIDCoefficients(.07,0,0);
    public PIDCoefficients moveYPID = new PIDCoefficients(.07,0,0);
    public double moveXSmoothingSteps = 1;
    public double moveYSmoothingSteps = 1;
    public double rotationSmoothingSteps = 1;

    public double speedMultiplierMin = .2;
    public double speedMultiplierMax = 2;

    //controls
    GamepadButtonManager XMoveStick = new GamepadButtonManager(GamepadButtons.leftJoyStickX);
    GamepadButtonManager YMoveStick = new GamepadButtonManager(GamepadButtons.leftJoyStickY);
    GamepadButtonManager RotMoveStick = new GamepadButtonManager(GamepadButtons.rightJoyStickX);

    //presets
    MoveToPositionSettings finalPosSettings = new MoveToPositionSettings(new double[]{.75, .75, .5}, 20, 10000, 1);
    MoveToPositionSettings losePosSettings = new MoveToPositionSettings(new double[]{4, 4, 7.5}, 1, 10000, 1);

    MovementSettings(){}
}

class MoveToPositionSettings
{
    double[] tol;
    int timesInTol;
    int maxRuntime;
    double maxPower;
    PIDCoefficients turnPID = null;
    PIDCoefficients xPID = null;
    PIDCoefficients yPID = null;

    MoveToPositionSettings(){}
    MoveToPositionSettings(double[] tol, int timesInTol, int maxRuntime, double maxPower)
    {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
    }

    MoveToPositionSettings(double[] tol, int timesInTol, int maxRuntime, double maxPower, PIDCoefficients xPID, PIDCoefficients yPID, PIDCoefficients turnPID)
    {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
        this.xPID = xPID;
        this.yPID = yPID;
        this.turnPID = turnPID;
    }

    public boolean isPIDValid()
    {
        return turnPID != null && xPID != null && yPID != null;
    }


    RotToAngleSettings toRotAngleSettings()
    {
        if(turnPID != null) return new RotToAngleSettings(tol[2], timesInTol, maxRuntime, maxPower, turnPID);
        return new RotToAngleSettings(tol[2], timesInTol, maxRuntime, maxPower);
    }


}

class RotToAngleSettings
{
    double tol;
    int timesInTol;
    int maxRuntime;
    double maxPower;
    PIDCoefficients turnPID;

    RotToAngleSettings(){}
    RotToAngleSettings(double tol, int timesInTol, int maxRuntime, double maxPower)
    {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
    }

    RotToAngleSettings(double tol, int timesInTol, int maxRuntime, double maxPower, PIDCoefficients turnPID)
    {
        this.tol = tol;
        this.timesInTol = timesInTol;
        this.maxRuntime = maxRuntime;
        this.maxPower = maxPower;
        this.turnPID = turnPID;
    }

    boolean isPIDValid()
    {
        return turnPID != null;
    }
}
