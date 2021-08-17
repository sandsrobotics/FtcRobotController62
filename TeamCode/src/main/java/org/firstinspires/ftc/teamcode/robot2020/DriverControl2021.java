package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Driver 2021")
public class DriverControl2021 extends LinearOpMode
{
    Robot robot;
    GamepadButtonManager speedToggle;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useDrive = true;
        ru.positionUsage = new PositionUsage(true,true,true,false,false);

        robot = new Robot(this, ru);

        waitForStart();

        speedToggle = new GamepadButtonManager(gamepad1, GamepadButtons.leftTRIGGER);
        speedToggle.minSliderVal = 0.3;

        robot.start(true);

        while (opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1, true);

            if(speedToggle.getButtonHeld()) robot.movement.setSpeedMultiplier(.3);
            else robot.movement.setSpeedMultiplier(1);

            robot.sendTelemetry();
        }
    }
}
