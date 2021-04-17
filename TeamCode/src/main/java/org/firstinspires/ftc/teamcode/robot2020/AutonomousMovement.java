package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// test
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "test auto move v1")
public class AutonomousMovement extends LinearOpMode {

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

    double minLaunchDistance = -64;

    Position[] positions = {
            new Position(24, minLaunchDistance, -7),
            new Position(24, minLaunchDistance, 0.5),
            new Position(24, minLaunchDistance, 7)
    };

    ///////////////////
    //other variables//
    ///////////////////
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
        ru.setAllToValue(false);
        ru.useDrive = true;
        ru.positionUsage.usePosition = true;
        ru.positionUsage.usePositionThread = true;
        ru.positionUsage.usePositionCamera = true;

        /////////
        //start//
        /////////
        robot = new Robot(this, ru);

        robot.startTelemetry();

        waitForStart();

        robot.start(false, true);

        ////////////////
        //main program//
        ////////////////

        robot.delay(2000);

        //move to base pos
        robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);

        //launch power shot
        for(int i = 0; i < 3; i++) {
            robot.movement.moveToPosition(positions[i], robot.movement.movementSettings.finalPosSettings);
        }

        robot.delay(1000);

        //drop goal one
        goToDropZone(finalNumOfRings, 1);

        robot.delay(1000);

        robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(0,-7, 0), robot.movement.movementSettings.losePosSettings);

        //get ready and go to second goal
        if(finalNumOfRings == 4) robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);

        if(finalNumOfRings != 1){
            robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.finalPosSettings);
        }
        else{
            robot.movement.moveToPosition(secondGoalPositions[1][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[1][1], robot.movement.movementSettings.finalPosSettings);
        }

        robot.delay(1000);

        //drop off second goal
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }

        goToDropZone(finalNumOfRings, 2);

        robot.delay(1000);

        robot.movement.moveToPosition(robot.positionTracker.getPositionWithOffset(3,-7, 0), robot.movement.movementSettings.losePosSettings);

        //park
        robot.movement.moveToPosition(parkPos,robot.movement.movementSettings.finalPosSettings);

        robot.positionTracker.writeRotationToFile();
    }

    void goToDropZone(int pos, int goalNum)
    {
        if(pos == 0) { robot.movement.moveToPosition(APositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 1) { robot.movement.moveToPosition(BPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 4) { robot.movement.moveToPosition(CPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
    }

}