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
        new Position(-20,-62,-90),
        new Position(-18,-68,-90)
    };
    Position[] BPositions = {
        new Position(-1,-38,-90),
        new Position(6,-48,-90)
    };
    Position[] CPositions = {
        new Position(-20,-14,-90), //-25
        new Position(-18,-20,-90)
    };

    Position[][] secondGoalPositions = {
        {
            new Position(-10, -115, 0),  //106
//            new Position(-2, -112, 0)
            new Position(-2, -115, 0)
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

        /////////
        //start//
        /////////
        robot = new Robot(this, ru);

        robot.positionTracker.positionSettings.startPosMode = 1;

        robot.vision.tofdActivationSequence();
        robot.startTelemetry();

        //lk changes
        robot.start(false, true);
        robot.positionTracker.waitForPositionInitialization(2000);

        while(!isStarted() && !isStopRequested())
        {
            if(closeButton.getButtonPressed(gamepad1)) {
                robot.grabber.setIntakeMode((short) 1);
                robot.grabber.moveServos();
            }
            else if(closeButton.getButtonReleased(gamepad1)){
                robot.grabber.setIntakeMode((short) 0);
                robot.grabber.moveServos();
            }

            calculatedNumOfRings = getNumOfRings();
            if(calculatedNumOfRings == -1) robot.addTelemetry("rings", " calculating...");
            else
            {
                finalNumOfRings = calculatedNumOfRings;
                robot.addTelemetry("rings", finalNumOfRings);
            }

            //debugging addition
            robot.positionTracker.drawAllPositions();
            robot.addTelemetry("dist", robot.positionTracker.distSensorPosition.toString(2));
            robot.addTelemetry("enc", robot.positionTracker.encoderPosition.toString(2));
            robot.addTelemetry("cam", robot.positionTracker.cameraPosition.toString(2));
            robot.addTelemetry("main pos", robot.positionTracker.currentPosition.toString(2));


            robot.sendTelemetry();
        }

        if(isStopRequested()) return;

        //robot.start(false, true);
        //robot.positionTracker.waitForPositionInitialization(2000);

        ////////////////
        //main program//
        ////////////////

        //move to base pos
        robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);

        //setup for launch
        //robot.grabber.setGrabberToPos(straitUpPos, false);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.straitUpPos, false);

        //launch power shot
        robot.launcher.autoLaunchPowerShots(robot.launcher.launcherSettings.powerShotPos);
        robot.launcher.setRPM(0);

        //drop goal one
        robot.robotUsage.positionUsage.useDistanceSensors = false;
        goToDropZone(finalNumOfRings, 1);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.runGrabberOuttake(true);
        //robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0,-7, 0), robot.movement.movementSettings.losePosSettings);

        //get ready and go to second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.robotUsage.positionUsage.useDistanceSensors = true;
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        if(finalNumOfRings != 1){
            robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.mediumPosSettings);
            robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.mediumPosSettings);
        }
        else{
            robot.movement.moveToPosition(secondGoalPositions[1][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[1][1], robot.movement.movementSettings.mediumPosSettings);
        }

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        //robot.grabber.runGrabberIntake();

        robot.grabber.setGrabberToIntake();

        if(finalNumOfRings==1) {
            robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(-6, 0, 0), robot.movement.movementSettings.capturePosSettings);
            robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(6, 0, 0), robot.movement.movementSettings.capturePosSettings);
        }
        else {
            robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(6, 0, 0), robot.movement.movementSettings.capturePosSettings);
            robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(-6, 0, 0), robot.movement.movementSettings.capturePosSettings);
        }


        robot.grabber.setGrabberToIntakeOff();

        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.drivePos, false);
        //robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.horizontalPos, false);

        //drop off second goal
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        //robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 70, false);
        goToDropZone(finalNumOfRings, 2);
        robot.grabber.runGrabberOuttake(true, 500);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);

        robot.robotUsage.positionUsage.useDistanceSensors = false;

        //robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(3,-7, 0), robot.movement.movementSettings.losePosSettings);

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
        if(pos == 0) { robot.movement.moveToPosition(APositions[goalNum - 1], robot.movement.movementSettings.mediumPosSettings); }
        else if(pos == 1) { robot.movement.moveToPosition(BPositions[goalNum - 1], robot.movement.movementSettings.mediumPosSettings); }
        else if(pos == 4) { robot.movement.moveToPosition(CPositions[goalNum - 1], robot.movement.movementSettings.mediumPosSettings); }
    }

}

