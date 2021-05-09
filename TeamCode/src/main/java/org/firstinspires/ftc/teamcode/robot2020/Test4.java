package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
// test
@Config
@TeleOp(name = "test complex movement")
public class Test4 extends LinearOpMode
{

    Robot robot;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useDrive = true;
        ru.useComplexMovement = true;

        robot = new Robot(this, ru);
        robot.robotHardware.setMotorsZeroPowerBehaviorList(robot.robotHardware.driveMotors, DcMotor.ZeroPowerBehavior.FLOAT);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        while(!robot.isStop() && opModeIsActive())
        {
            robot.startTelemetry();

            if(robot.complexMovement.isRecording)
            {
                robot.addTelemetry("Robot: ", "recording");
                robot.sendTelemetry();
                while (robot.complexMovement.isRecording && !robot.isStop()) { robot.complexMovement.recorder(true); }
                robot.addTelemetry("Robot: ", "done recording");
                robot.sendTelemetry();
                robot.complexMovement.stopRecording(true, "test");
                robot.addTelemetry("Robot: ", "ready to move");
                robot.sendTelemetry();
            }

            if(gamepad1.a)
            {
                robot.startTelemetry();
                robot.complexMovement.loadMoveDB("test");
                robot.complexMovement.runLoadedMoveV2(false);
                robot.addTelemetry("Robot: ", "done with move");
                robot.sendTelemetry();
            }
            else if(gamepad1.b && gamepad1.x)
            {
                robot.startTelemetry();
                robot.addTelemetry("database clear activated: ", "hold B for 2 second to clear");
                robot.sendTelemetry();
                sleep(2000);
                if(gamepad1.b)
                {
                    robot.complexMovement.clearDatabase();
                    robot.startTelemetry();
                    robot.addTelemetry("database cleared", "");
                    robot.sendTelemetry();
                    sleep(1000);
                }
                else
                {
                    robot.startTelemetry();
                    robot.addTelemetry("database clear deactivated: ", "");
                    robot.sendTelemetry();
                }
            }
            else if(gamepad1.y) robot.complexMovement.startRecording();
        }
    }
}
