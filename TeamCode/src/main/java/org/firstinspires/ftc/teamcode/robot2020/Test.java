package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test position tracking")
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
        ru.positionUsage.usePositionCamera = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true, false);

        while (opModeIsActive())
        {
           robot.movement.moveForTeleOp(gamepad1, brake, false);
           robot.addTelemetry("X", robot.positionTracker.currentPosition.X);
           robot.addTelemetry("Y", robot.positionTracker.currentPosition.Y);
           robot.addTelemetry("R", robot.positionTracker.currentPosition.R);
           robot.sendTelemetry();
        }
    }
}
