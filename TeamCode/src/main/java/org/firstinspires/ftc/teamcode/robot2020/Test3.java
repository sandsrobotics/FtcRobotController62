package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
// test
@Config
@TeleOp(name = "test second goal pickup")
public class Test3 extends LinearOpMode {

    Robot robot;

    Position basePos = new Position(-20, -80, 0);

    Position[][] secondGoalPositions = {
        {
            new Position(-10,-106, 0),
            new Position(-2, -112, 0)
        }, {
            new Position(30, -100, 180),
            new Position(22, -108.5, 180)
        }
    };

    int straitUpPos = 500;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.useComplexMovement = false;
        ru.visionUsage.useTensorFlowInTread = false;
        ru.visionUsage.useOpenCV = false;
        ru.visionUsage.useVuforiaInThread = false;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(false, false);

        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.robotUsage.positionUsage.useDistanceSensors = true;
        robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.losePosSettings);
        robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.finalPosSettings);

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.delay(2000);
        //robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, true);

        //drop off second goal
        robot.grabber.setGrabberToPos(straitUpPos, false);

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1, true);
        }
    }
}