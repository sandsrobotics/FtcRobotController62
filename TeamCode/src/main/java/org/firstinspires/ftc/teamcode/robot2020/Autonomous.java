package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// test
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "test auto v1")
public class Autonomous extends LinearOpMode {

    //////////////////
    //user variables//
    //////////////////
    //buttons
    GamepadButtonManager closeButton = new GamepadButtonManager(GamepadButtons.A);

    //positions
    Position basePos = new Position(-20, -80, 0);

    Position[] APositions = {
        new Position(-25,-62,-90),
        new Position(-18,-68,-90)
    };
    Position[] BPositions = {
        new Position(-1,-38,-90),
        new Position(6,-48,-90)
    };
    Position[] CPositions = {
        new Position(-25,-14,-90),
        new Position(-18,-20,-90)
    };

    Position[][] secondGoalPositions = {
        {
            new Position(-10, - 106, 0),
            new Position(-2, -112, 0)
        }, {
            new Position(30, -100, 180),
            new Position(22, -107.5, 180)
        }
    };

    Position parkPos = new Position(0,-53,-90);

    //other
    int timesRingRecognitionReq = 10; //how many times does tfod have to see a certain number of rings to call it good

    ///////////////////
    //other variables//
    ///////////////////
    //grabber
    boolean closed = true;
    int straitUpPos = 500;

    //tfod
    int timesRingsRecognized = 0;
    int lastNumOfRings = -1;

    int calculatedNumOfRings;
    int finalNumOfRings = -1; //what is the final say on the number of rings

    //other
    Robot robot;


    ////////
    //code//
    ////////
    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.useComplexMovement = false;

        ru.visionUsage.useTensorFlowInTread = false;
        ru.visionUsage.useOpenCV = false;
        ru.visionUsage.useVuforiaInThread = false;
        
        ru.positionUsage.usePositionCamera = false;

        /////////
        //start//
        /////////
        robot = new Robot(this, ru);

        robot.positionTracker.positionSettings.startPosMode = 1;

        robot.vision.tofdActivationSequence();
        robot.startTelemetry();

        while(!isStarted() && !isStopRequested())
        {
            if(closeButton.getButtonPressed(gamepad1))
            {
                robot.grabber.runGrabberIntake();
            }

            calculatedNumOfRings = getNumOfRings();
            if(calculatedNumOfRings == -1) robot.addTelemetry("rings", " calculating...");
            else
            {
                finalNumOfRings = calculatedNumOfRings;
                robot.addTelemetry("rings", finalNumOfRings);
            }
            robot.sendTelemetry();
        }

        if(isStopRequested()) return;

        robot.start(false, true);
        robot.positionTracker.waitForPositionInitialization();

        ////////////////
        //main program//
        ////////////////

        //move to base pos
        robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);

        //setup for launch
        robot.grabber.setGrabberToPos(straitUpPos, false);

        //launch power shot
        robot.launcher.autoLaunchPowerShots(robot.launcher.launcherSettings.powerShotPos);
        robot.launcher.setRPM(0);

        //drop goal one
        robot.robotUsage.positionUsage.useDistanceSensors = false;
        goToDropZone(finalNumOfRings, 1);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.runGrabberIntake();
        robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0,-7, 0), robot.movement.movementSettings.losePosSettings);

        //get ready and go to second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.robotUsage.positionUsage.useDistanceSensors = true;
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        if(finalNumOfRings != 1){
            robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.finalPosSettings);
        }
        else{
            robot.movement.moveToPosition(secondGoalPositions[1][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[1][1], robot.movement.movementSettings.finalPosSettings);
        }

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.runGrabberIntake();
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 70, false);

        //drop off second goal
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 70, false);
        goToDropZone(finalNumOfRings, 2);
        robot.grabber.runGrabberIntake();
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);

        robot.robotUsage.positionUsage.useDistanceSensors = false;

        robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(3,-7, 0), robot.movement.movementSettings.losePosSettings);

        //park
        robot.movement.moveToPosition(parkPos,robot.movement.movementSettings.finalPosSettings);

        robot.positionTracker.writePositionToFile();
    }


    ///////////
    //methods//
    ///////////
    int getNumOfRings()
    {
        int currentNumOfRings = robot.vision.runTfodSequenceForRings();
        if(currentNumOfRings == lastNumOfRings)
        {
            timesRingsRecognized++;
            if(timesRingsRecognized >= timesRingRecognitionReq){ return currentNumOfRings;}
        }
        else
        {
            timesRingsRecognized = 0;
            lastNumOfRings = currentNumOfRings;
        }
        return -1;
    }

    void goToDropZone(int pos, int goalNum)
    {
        if(pos == 0) { robot.movement.moveToPosition(APositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 1) { robot.movement.moveToPosition(BPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 4) { robot.movement.moveToPosition(CPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
    }

}

