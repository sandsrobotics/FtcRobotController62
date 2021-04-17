package org.firstinspires.ftc.teamcode.robot2020;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "tune PIDF")
@Config
public class Test2 extends LinearOpMode
{
    Robot robot;
    GamepadButtonManager breakButton = new GamepadButtonManager(GamepadButtons.leftJoyStickBUTTON);
    GamepadButtonManager autoLaunchButton;
    public static PIDFCoefficients PIDF = new PIDFCoefficients(1.3,.13,0,12);

    short mode = 0;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.visionUsage.useVuforia = false;
        ru.useComplexMovement = false;
        ru.visionUsage.useTensorFlow = false;
        ru.positionUsage.useDistanceSensors = false;
        ru.useGrabber = false;
        ru.visionUsage.useOpenCV = false;

        robot = new Robot(this,ru);

        waitForStart();

        autoLaunchButton = new GamepadButtonManager(gamepad1, GamepadButtons.dpadUP);
        robot.start(true, false);

        while (opModeIsActive())
        {
            robot.robotHardware.launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            if(mode == 0)
            {
                robot.movement.moveForTeleOp(gamepad1, breakButton, true);
                robot.launcher.runForTeleOp(gamepad2,true);
                robot.addTelemetry("min", 0);
                if(autoLaunchButton.getButtonHeld()) mode = 1;
                robot.sendTelemetry();
            }
            else if(mode == 1)
            {
                robot.launcher.goToLine();
                mode = 0;
            }
        }
    }
}
