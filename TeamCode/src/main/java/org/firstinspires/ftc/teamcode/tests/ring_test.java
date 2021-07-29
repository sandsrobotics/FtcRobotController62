package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ring_test (Blocks to Java)", group = "")
@Disabled
public class ring_test extends LinearOpMode {

    private Servo servo0;
    private DcMotorEx motor0;
    private DcMotorEx motor1;
    private CRServo servo3;
    private DcMotorEx motor3;
    private DcMotorEx motor2;
    private Servo servo1;

    double spin_power2;
    int launch45;
    int launcherPos;
    double servolaunch;
    double servoneutral;
    int launcherHome;
    boolean SM;
    boolean mm;

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_stuff();
        variables();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                launcher();
                standard_drive();
                spin_power();
                launch();
                sweep();
                data_out();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void launch() {
        if (gamepad1.right_bumper) {
            servo0.setPosition(servolaunch);
        } else {
            servo0.setPosition(servoneutral);
        }
    }

    /**
     * Describe this function...
     */
    private void standard_drive() {
        motor0.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        motor1.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
    }

    /**
     * Describe this function...
     */
    private void sweep() {
        if (gamepad1.left_bumper) {
            servo3.setPower(0.5);
        } else {
            servo3.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void bala_drive() {

        if (SM) {
            motor0.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) + gamepad1.left_stick_x) / 4);
            motor1.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) - gamepad1.left_stick_x) / 4);
        } else if (mm) {
            motor0.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) + gamepad1.left_stick_x) / 2);
            motor1.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) - gamepad1.left_stick_x) / 2);
        } else {
            motor0.setPower((gamepad1.right_trigger - gamepad1.left_trigger) + gamepad1.left_stick_x);
            motor1.setPower((gamepad1.right_trigger - gamepad1.left_trigger) - gamepad1.left_stick_x);
        }
    }

    /**
     * Describe this function...
     */
    private void motor_stuff() {
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setDirection(DcMotorEx.Direction.FORWARD);
        motor3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        // servo0.setDirection(Servo.Direction.REVERSE);
        servo3.setDirection(DcMotorSimple.Direction.REVERSE);
        servo0.setPosition(0.5);
        servo1.setPosition(1);
        servo1.setPosition(1);
    }

    /**
     * Describe this function...
     */
    private void spin_power() {
        spin_power2 = -gamepad1.right_stick_y;
        if (spin_power2 < 0) {
            spin_power2 = 0;
        }
        // motor3.setVelocity(90.0);
        motor3.setPower(spin_power2);
    }

    /**
     * Describe this function...
     */
    private void launcher() {
        if (gamepad1.a) {
            launcherPos = launcherHome;
        }
        if (gamepad1.b) {
            launcherPos = launch45;
        }
        if (gamepad1.y) {
            launcherPos = -500;
        }
        motor2.setTargetPosition(launcherPos);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.5);
    }

    /**
     * Describe this function...
     */
    private void data_out() {
        // where spinRPM is my target speed, 145.6 is the motor's ticks/rev, and 7 is my
        // gear ratio.
        // Double spinMultiplier = 60 / 145.6 * 5;
        Double spinMultiplier = 60 / 28.0;
        Double spinSpeed;
        spinSpeed = motor3.getVelocity() * spinMultiplier;

        telemetry.addData("spinSpeed", spinSpeed);
        telemetry.addData("Spin power", motor3.getPower());
        telemetry.addData("Ramp", motor2.getCurrentPosition());
        telemetry.addData("Sweep", servo3.getPower());
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void variables() {
        double all_the_way_down;
        double my_90_degrees;
        double straight_up;

        launch45 = -600;
        launcherPos = 0;
        spin_power2 = 0;
        all_the_way_down = -1000;
        my_90_degrees = -800;
        straight_up = -420;
        launcherHome = 0;
        servoneutral = 0.55;
        servolaunch = 0.25;
    }
}