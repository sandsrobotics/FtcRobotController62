package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot2020.GamepadButtonManager;
import org.firstinspires.ftc.teamcode.robot2020.GamepadButtons;
import org.firstinspires.ftc.teamcode.robot2020.Robot;
import org.firstinspires.ftc.teamcode.robot2020.RobotUsage;
@Disabled
@Config
@TeleOp(name = "test positionTracker tracking camera")
public class Test5 extends LinearOpMode {

    Robot robot;
    GamepadButtonManager brake = new GamepadButtonManager(GamepadButtons.A);

    // This is the transformation between the center of the camera and the center of the robot
    static Transform2d cameraToRobot = new Transform2d();
    static double encoderMeasurementCovariance = 0.8;
    Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());
    static T265Camera slamra;


    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true, false);
        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
        slamra.start();

        while (opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1, brake, false);
            T265Camera.CameraUpdate cu = slamra.getLastReceivedCameraUpdate();
            robot.addTelemetry("X", cu.pose.getTranslation().getX());
            robot.addTelemetry("Y", cu.pose.getTranslation().getY());
            robot.addTelemetry("R", cu.pose.getRotation().getDegrees());
            robot.sendTelemetry();
        }
    }
}
