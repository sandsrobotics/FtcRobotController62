package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

@Config
@TeleOp(name = "test Optimized Ultrasonic")
public class TestUltra extends LinearOpMode {
    public static int d0 = 20;
    public static int d1 = 20;

    Robot robot;
    GamepadButtonManager brake = new GamepadButtonManager(GamepadButtons.A);

    int sensor = 0;
    Position distSensorPosition = new Position();

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useDrive = true;

        // setup task based on background thread at bottom
        TimerTask task = new ScheduleTask();
        // create timer that can later be scheduled
        Timer tmr = new Timer();

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true, false);
        robot.robotHardware.initUltrasonicSensors();
        robot.startTelemetry();

        // schedule repeat task at a period of 20 ms, delay is time before first task
        tmr.scheduleAtFixedRate(task, 0, 20);

        while (opModeIsActive())
        {
            robot.startTelemetry();
            robot.movement.moveForTeleOp(gamepad1, brake, false);

            distSensorPosition.X = Math.floor(robot.robotHardware.distSensors.get(0).getDistanceIn() *10) / 10.0;
            distSensorPosition.Y = Math.floor(robot.robotHardware.distSensors.get(1).getDistanceIn() *10) / 10.0;

            robot.addTelemetry("X", distSensorPosition.X);
            robot.addTelemetry("Y", distSensorPosition.Y);
            robot.sendTelemetry();
        }
    }

    class ScheduleTask extends TimerTask {
        public void run() {
            robot.robotHardware.distSensors.get(sensor).measureRange();
            sensor = 1 - sensor;  // to flip between sensor 0 and 1 for next run
        }
    }
}
