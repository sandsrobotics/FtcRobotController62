package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotHardware
{

    /////////
    //other//
    /////////
    //drive
    protected DcMotorEx leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    protected List<DcMotorEx> driveMotors;

    //launcher
    protected DcMotorEx launcherWheelMotor, launcherIntakeMotor;
    protected Servo launcherLaunchServo;
    protected Servo launcherGateServo;
    //protected CRServo launcherIntakeServo;

    //grabber
    protected DcMotorEx grabberLifterMotor;
    //protected Servo grabberLeftServo, grabberRightServo;
    protected CRServo grabberLeftServo, grabberRightServo;
    //protected List<Servo> grabberServos;
    protected List<CRServo> grabberServos;
    protected DigitalChannel grabberArmLimitSwitch;
    protected DigitalChannel grabberIntakeLimitSwitch;

    //ultrasonic
    protected List<DFR304Range> distSensors;
    protected DFR304Range distSensor1;
    //protected DFR304Range distSensorX2;
    protected DFR304Range distSensor2;

    //other class
    Robot robot;
    HardwareSettings hardwareSettings;

    public RobotHardware(Robot robot)
    {
        hardwareSettings = new HardwareSettings();
        this.robot = robot;
    }
    public RobotHardware(Robot robot, HardwareSettings hardwareSettings)
    {
        this.hardwareSettings = hardwareSettings;
        this.robot = robot;
    }

    ////////
    //init//
    ////////
    public void initDriveMotors()
    {
        leftTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + hardwareSettings.leftTopMotorNum);
        leftBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + hardwareSettings.leftBottomMotorNum);
        rightTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + hardwareSettings.rightTopMotorNum);
        rightBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + hardwareSettings.rightBottomMotorNum);
        driveMotors = Arrays.asList(leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor);

        int i = 0;
        for(DcMotor motor:driveMotors)
        {
            if(hardwareSettings.flipDriveMotorDir[i]) motor.setDirection(DcMotor.Direction.REVERSE);
            i++;
        }

        initMotorSettings(driveMotors, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initLauncherMotors()
    {
        launcherWheelMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + hardwareSettings.launcherWheelMotorNum);
        launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, HardwareSettings.launcherWheelMotorPID);
        launcherIntakeMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + hardwareSettings.launcherIntakeMotorNum);

        launcherLaunchServo = robot.hardwareMap.servo.get("servo" + hardwareSettings.launcherLaunchServoNum);
        launcherGateServo = robot.hardwareMap.servo.get("servo" + hardwareSettings.launcherGateServoNum);
        //launcherIntakeServo = robot.hardwareMap.crservo.get("servo" + hardwareSettings.launcherIntakeServoNum);

        if(hardwareSettings.flipLauncherMotorDir[0]) launcherWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        if(hardwareSettings.flipLauncherMotorDir[1]) launcherIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        if(hardwareSettings.flipLauncherMotorDir[2]) launcherLaunchServo.setDirection(Servo.Direction.REVERSE);
        if(hardwareSettings.flipLauncherMotorDir[3]) launcherGateServo.setDirection(Servo.Direction.REVERSE);
        //if(hardwareSettings.flipLauncherMotorDir[3]) launcherIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        initMotorSettings(launcherWheelMotor, DcMotor.ZeroPowerBehavior.FLOAT);

        launcherIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherLaunchServo.setPosition(robot.launcher.launcherSettings.launcherServoRestAngle);
        launcherGateServo.setPosition(robot.launcher.launcherSettings.gateServoRestAngle);
        //launcherIntakeServo.setPower(0);
    }

    public void initGrabberHardware()
    {
        grabberLifterMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + hardwareSettings.grabberLifterMotorNum);
        grabberLeftServo = robot.hardwareMap.crservo.get("servo" + hardwareSettings.grabberLeftServoNum);
        grabberRightServo = robot.hardwareMap.crservo.get("servo" + hardwareSettings.grabberRightServoNum);
        grabberServos = Arrays.asList(grabberLeftServo, grabberRightServo);

        if(hardwareSettings.flipGrabberMotorDir[0]) grabberLifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(hardwareSettings.flipGrabberMotorDir[1]) grabberServos.get(0).setDirection(CRServo.Direction.REVERSE);
        if(hardwareSettings.flipGrabberMotorDir[2]) grabberServos.get(1).setDirection(CRServo.Direction.REVERSE);

        grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberLifterMotor.setTargetPosition(0);
        grabberLifterMotor.setPower(robot.grabber.grabberSettings.motorPower);
        grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberArmLimitSwitch = robot.hardwareMap.get(DigitalChannel.class, hardwareSettings.grabberArmLimitSwitchName);
        grabberArmLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        grabberIntakeLimitSwitch = robot.hardwareMap.get(DigitalChannel.class, hardwareSettings.grabberIntakeLimitSwitchName);
        grabberIntakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initUltrasonicSensors()
    {
        distSensor1 = robot. hardwareMap.get(DFR304Range.class, "distSensor" + hardwareSettings.Ultrasonic1Num);
       // distSensorX2 = robot. hardwareMap.get(DFR304Range.class, "distSensor" + hardwareSettings.X2UltrasonicNum);
        distSensor2 = robot. hardwareMap.get(DFR304Range.class, "distSensor" + hardwareSettings.Ultrasonic2Num);
        //distSensors = Arrays.asList(distSensor1,distSensorX2,distSensor2);
        distSensors = Arrays.asList(distSensor1, distSensor2);

        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM500;
        parameters.measureMode = DFR304Range.MeasureMode.PASSIVE;
        for(DFR304Range distSen : distSensors) { distSen.initialize(parameters); }
    }

    public void initMotorSettings(List<DcMotorEx> motors, DcMotor.ZeroPowerBehavior zeroPowerBehavior) { for(DcMotorEx motor:motors) initMotorSettings(motor, zeroPowerBehavior); }

    public void initMotorSettings(DcMotorEx motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetMotorEncodersList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    ///////////////////
    //set motor modes//
    ///////////////////
    public void setMotorsZeroPowerBehaviorList(List<DcMotorEx> motors, DcMotor.ZeroPowerBehavior zeroPowerBehavior) { for(DcMotorEx motor: motors) motor.setZeroPowerBehavior(zeroPowerBehavior); }

    public void setMotorsRunModeList(List<DcMotorEx> motors, DcMotor.RunMode runMode) { for(DcMotorEx motor: motors) motor.setMode(runMode); }

    ///////////////
    //motor power//
    ///////////////
    public void setMotorsToPowerList(List<DcMotorEx> motors, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setPower(power);
        }
    }

    public void setMotorsToSeparatePowersArrayList(List<DcMotorEx> motors, double[] powers)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setPower(powers[i]);
            i++;
        }
    }

    ///////////////
    //motor ticks//
    ///////////////
    public void setMotorsToPositionList(List<DcMotorEx> motors, int ticks, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setMotorsToSeparatePositionsAndPowersList(List<DcMotorEx> motors, int[] ticks, double[] power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(ticks[i]);
            motor.setPower(power[i]);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public void moveMotorsForwardList(List<DcMotorEx> motors, int ticks, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorForwardSeparateAmountList(List<DcMotorEx> motors, int[] ticks, double power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public void moveMotorForwardSeparateAmountsAndPowersList(List<DcMotorEx> motors, int[] ticks, double[] power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power[i]);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public int[] getMotorPositionsList(List<DcMotorEx> motors)
    {
        int[] arr = new int[motors.size()];
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            arr[i] = motor.getCurrentPosition();
            i++;
        }
        return arr;
    }

    //////////
    //servos//
    //////////
    void setServosPower(List<CRServo> servos, double power){
        for(CRServo servo : servos) servo.setPower(power);
    }

    /////////
    //other//
    /////////
    public float[] getDistancesList(List<DFR304Range> distSensors)
    {
        float[] arr = new float[distSensors.size()];
        for(int i = 0; i < distSensors.size(); i++)
        {
            if(distSensors.get(i).getParameters().measureMode == DFR304Range.MeasureMode.PASSIVE) {
                distSensors.get(i).measureRange();
            }
            arr[i] = distSensors.get(i).getDistanceIn();
        }
        return arr;
    }

    public float getDistance(DFR304Range distSensor)
    {
        if(distSensor.getParameters().measureMode == DFR304Range.MeasureMode.PASSIVE) { distSensor.measureRange(); }
        return distSensor.getDistanceIn();
    }

    public void setSensorsToMeasure(List<DFR304Range> distSensors)
    {
        for(int i = 0; i < distSensors.size(); i++) { distSensors.get(i).measureRange();}
    }

    public float[] getDistancesAfterMeasure(List<DFR304Range> distSensors)
    {
        float[] arr = new float[distSensors.size()];
        for(int i = 0; i < distSensors.size(); i++) { arr[i] = distSensors.get(i).getDistanceIn(); }
        return arr;
    }

    public void waitForMotorsToFinishList(List<DcMotorEx> motors)
    {
        int totalMotorsDone = 0;
        while(totalMotorsDone < motors.size())
        {
            totalMotorsDone = 0;
            if (robot.stop()) break;
            for (DcMotorEx motor : motors)
            {
                if(!motor.isBusy()) totalMotorsDone++;
            }
        }
    }

    public boolean motorPositionsInToleranceList(List<DcMotorEx> motors, int tolerance)
    {
        for (DcMotorEx m:motors){if(Math.abs(m.getTargetPosition()-m.getCurrentPosition()) > tolerance) return false;}
        return true;
    }
}

class  HardwareSettings
{
    //////////////////
    //user variables//
    //////////////////
    //drive motors
    protected boolean[] flipDriveMotorDir = {true, true, false, false};
    protected String leftTopMotorNum = "0";
    protected String leftBottomMotorNum = "2";
    protected String rightTopMotorNum = "1";
    protected String rightBottomMotorNum = "3";

    //launcher motors
    protected boolean[] flipLauncherMotorDir = {true, true, false, false};
    protected String launcherWheelMotorNum = "0B";
    public static PIDFCoefficients launcherWheelMotorPID = new PIDFCoefficients(100,0,0,12.4);
    protected String launcherIntakeMotorNum = "1B";
    protected String launcherLaunchServoNum = "0B";
    protected String launcherGateServoNum = "4B";

    //grabber motors
    protected boolean[] flipGrabberMotorDir = {true, false, true};
    protected String grabberLifterMotorNum = "2B";
    protected String grabberLeftServoNum = "1B";
    protected String grabberRightServoNum = "2B";
    protected String grabberArmLimitSwitchName = "digital0B";
    protected String grabberIntakeLimitSwitchName = "digital1B";

    //ultrasonic
    protected String Ultrasonic1Num = "0B";
    protected String X2UltrasonicNum = "2";
    protected String Ultrasonic2Num = "1B";

    HardwareSettings(){}
}