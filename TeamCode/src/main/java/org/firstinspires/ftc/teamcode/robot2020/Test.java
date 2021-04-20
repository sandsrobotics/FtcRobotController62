package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test all position trackers")
public class Test extends LinearOpMode {

    Robot robot;
    GamepadButtonManager brake = new GamepadButtonManager(GamepadButtons.A);

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.positionUsage.usePositionThread = true;
        ru.positionUsage.usePosition = true;
        ru.positionUsage.useDistanceSensors = true;
        ru.positionUsage.useEncoders = true;
        ru.positionUsage.usePositionCamera = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true, false);

        while (opModeIsActive())
        {
            robot.startTelemetry();
            robot.movement.moveForTeleOp(gamepad1, brake, false);
            if(robot.positionTracker.drawDashboardField) robot.positionTracker.drawAllPositions();
            robot.sendTelemetry();
        }
    }
}
