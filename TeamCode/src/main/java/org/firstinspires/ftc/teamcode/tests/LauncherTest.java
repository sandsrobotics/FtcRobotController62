package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test launcher")
@Disabled
public class LauncherTest extends LinearOpMode {

    //////////////////
    // user variables//
    //////////////////
    // pin num
    protected int launcherWheelMotorNum = 3;
    protected int launcherLifterMotorNum = 2;
    protected int launcherservoNum = 0;
    // flip
    protected boolean fliplauncherWheelMotor = false;
    protected boolean fliplauncherLifterMotor = false;
    protected boolean fliplauncherservo = false;
    // servo
    protected double servoRestAngle = .25;
    protected double servoLaunchAngle = .55;
    // wheel
    protected double gearRatio = 5;
    protected double ticksPerRev = 145.6;
    protected double rpmIncrements = 100;
    protected double maxRpm = 6000;

    // Use Encoder for Launcher wheel
    protected boolean useEncoder = true;
    // lifter
    protected double ticksPerDegree = 2.0 * 1120.0 / 360.0; // Andymark 40:1 2:1 chain gear ratio
    protected double maxAngle = 90;
    protected double rotationIncrements = 10;
    protected boolean resetLifterDuringStart = true;

    /////////
    // other//
    /////////
    protected double setLifterAngle = 0;
    protected double chainLashOffset = 9;
    protected double initialLifterOffsetAngle = -30;
    protected double setWheelRpm = 0;
    protected Boolean runWheelOnTrigger = true;
    protected FtcDashboard dashboard;
    Double spinMultiplier;
    Double spinVelocity;
    boolean b_pressed, x_pressed, y_pressed, dl_pressed, dr_pressed = false;
    int numOfTimeBPressed = 0;
    int numOfTimeXPressed = 0;

    TelemetryPacket packet;

    DcMotorEx launcherWheelMotor;
    DcMotor launcherLifterMotor;
    Servo launcherServo;

    @Override
    public void runOpMode() {
        initStuff();

        waitForStart();

        while (opModeIsActive()) {
            setLauncherSevo();
            setLauncherWheelMotor();
            setLauncherLifterMotor();
            getInputs();
            data_out();
        }
        // set ramp down easy
        setLifterAngle = initialLifterOffsetAngle + 10;
        setLauncherLifterMotor();
    }

    void initStuff() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // initiate
        launcherWheelMotor = hardwareMap.get(DcMotorEx.class, "motor" + launcherWheelMotorNum);
        launcherLifterMotor = hardwareMap.dcMotor.get("motor" + launcherLifterMotorNum);
        launcherServo = hardwareMap.servo.get("servo" + launcherservoNum);

        // reverse
        if (fliplauncherWheelMotor)
            launcherWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        if (fliplauncherLifterMotor)
            launcherWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        if (fliplauncherservo)
            launcherServo.setDirection(Servo.Direction.REVERSE);

        // zero motors
        launcherWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherServo.setPosition(servoRestAngle);
        if (resetLifterDuringStart)
            resetLifter();

        // set motor modes
        if (useEncoder)
            launcherWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            launcherWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set multiplier
        spinMultiplier = 60 / ticksPerRev * gearRatio;
    }

    private void data_out() {
        double RPM = launcherWheelMotor.getVelocity() * spinMultiplier;
        telemetry.addData("RPM", RPM);
        telemetry.addData("Set RPM", setWheelRpm);
        // 0 encoder = ? Degrees
        telemetry.addData("Ramp Encoder", launcherLifterMotor.getCurrentPosition());
        telemetry.addData("Ramp Angle", setLifterAngle);
        telemetry.update();

        packet.put("RPM: ", RPM);
        dashboard.sendTelemetryPacket(packet);
    }

    void resetLifter() {
        // later
    }

    void setLauncherSevo() {
        if (gamepad1.right_bumper)
            launcherServo.setPosition(servoLaunchAngle);
        else
            launcherServo.setPosition(servoRestAngle);
    }

    void setLauncherWheelMotor() {
        if (runWheelOnTrigger)
            launcherWheelMotor.setPower(gamepad1.left_trigger);
        else
            launcherWheelMotor.setVelocity(setWheelRpm / spinMultiplier);
    }

    void setLauncherLifterMotor() {
        launcherLifterMotor.setTargetPosition(
                (int) -((setLifterAngle - initialLifterOffsetAngle + chainLashOffset) * ticksPerDegree));
        launcherLifterMotor.setPower(.3);
        launcherLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void getInputs() {
        if (gamepad1.y) {
            if (!y_pressed) {
                y_pressed = true;
                runWheelOnTrigger = !runWheelOnTrigger;
            }
        } else
            y_pressed = false;

        if (gamepad1.b) {
            numOfTimeBPressed++;
            if (!b_pressed) {
                setWheelRpm += rpmIncrements;
                b_pressed = true;
            } else if (numOfTimeBPressed > 20)
                setWheelRpm += rpmIncrements;
        } else {
            b_pressed = false;
            numOfTimeBPressed = 0;
        }

        if (gamepad1.x) {
            numOfTimeXPressed++;
            if (!x_pressed) {
                x_pressed = true;
                setWheelRpm -= rpmIncrements;
            } else if (numOfTimeXPressed > 20)
                setWheelRpm -= rpmIncrements;
        } else {
            x_pressed = false;
            numOfTimeXPressed = 0;
        }

        if (setWheelRpm > maxRpm)
            setWheelRpm = maxRpm;
        if (setWheelRpm < 0)
            setWheelRpm = 0;

        if (gamepad1.dpad_right) {
            if (!dr_pressed) {
                dr_pressed = true;
                setLifterAngle += rotationIncrements;
            }
        } else
            dr_pressed = false;

        if (gamepad1.dpad_left) {
            if (!dl_pressed) {
                dl_pressed = true;
                setLifterAngle -= rotationIncrements;
            }
        } else
            dl_pressed = false;

        if (setLifterAngle < 0)
            setLifterAngle = 0;
        else if (setLifterAngle > maxAngle)
            setLifterAngle = maxAngle;

    }
}