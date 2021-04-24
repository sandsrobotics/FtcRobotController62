package org.firstinspires.ftc.teamcode.robot2020;

import androidx.room.Room;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot2020.persistence.AppDatabase;

import static java.lang.Thread.currentThread;


@Config
public class Robot
{
    ///////////////////
    //other variables//
    ///////////////////
    //other classes
    public RobotHardware robotHardware; //stores and configures all motors and servos
    public Movement movement;
    public Vision vision;
    public Launcher launcher;
    public ComplexMovement complexMovement;
    public PositionTracker positionTracker;
    public Grabber grabber;

    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected BNO055IMU imu;
    protected LinearOpMode opMode;
    protected AppDatabase db;
    protected RobotUsage robotUsage;
    protected RobotSettings robotSettings;

    //other
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    TelemetryPacket packet = new TelemetryPacket();
    public static boolean emergencyStop = false;

    Robot(LinearOpMode opMode, RobotUsage robotUsage, RobotSettingsMain robotSettingsMain) { init(opMode,robotUsage,robotSettingsMain); }
    Robot(LinearOpMode opMode, RobotUsage robotUsage) { init(opMode, robotUsage, new RobotSettingsMain()); }
    Robot(LinearOpMode opMode) { init(opMode, new RobotUsage(), new RobotSettingsMain()); }

    private void init(LinearOpMode opMode, RobotUsage robotUsage, RobotSettingsMain robotSettingsMain)
    {
        this.robotUsage = robotUsage;
        this.robotSettings = robotSettingsMain.robotSettings;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        robotHardware = new RobotHardware(this, robotSettingsMain.hardwareSettings);

        if(robotUsage.positionUsage.usePosition) positionTracker = new PositionTracker(this, robotSettingsMain.positionSettings);
        if(robotUsage.useDrive) movement = new Movement(this, robotSettingsMain.movementSettings);
        if(robotUsage.visionUsage.useVision) vision = new Vision(this, robotSettingsMain.visionSettings);
        if(robotUsage.useLauncher) launcher = new Launcher(this, robotSettingsMain.launcherSettings);
        if(robotUsage.useComplexMovement) complexMovement = new ComplexMovement(this);
        if(robotUsage.useGrabber){ grabber = new Grabber(this, robotSettingsMain.grabberSettings);
        addTelemetry("grabber", " init");}

        initHardware();
        if(robotUsage.useDrive || (robotUsage.positionUsage.usePosition && robotUsage.positionUsage.useEncoders) || robotUsage.useComplexMovement) robotHardware.initDriveMotors();
        if(robotUsage.useLauncher) robotHardware.initLauncherMotors();
        if(robotUsage.visionUsage.useVision) vision.initAll();
        if(robotUsage.useGrabber) robotHardware.initGrabberHardware();
        if(robotUsage.positionUsage.usePosition && robotUsage.positionUsage.useDistanceSensors) { robotHardware.initUltrasonicSensors(); }
    }

    void initHardware()
    {
        ///////////
        //sensors//
        ///////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())
        {
            delay(50);
            opMode.idle();
        }

        /////////////
        //dashboard//
        /////////////
        if(robotSettings.debug_dashboard) dashboard = FtcDashboard.getInstance();
        startTelemetry();

        /////////////
        //data base//
        /////////////
        db = Room.databaseBuilder(AppUtil.getDefContext(), AppDatabase.class, robotSettings.dataBaseName).build();
    }

    //------------------My Methods------------------//

    void start(boolean resetGrabberPos, boolean isAuto)
    {
        startTelemetry();
        if(robotUsage.positionUsage.useThread()) positionTracker.start();
        if(robotUsage.visionUsage.useThread()) vision.start();

        if(robotUsage.useGrabber && resetGrabberPos) grabber.initGrabberPos();
        if(!isAuto && robotUsage.useLauncher) launcher.initFrogLegs();
    }

    /////////////
    //telemetry//
    /////////////

    void startTelemetry()
    {
        if(robotSettings.debug_dashboard)
        {
            packet = new TelemetryPacket();
        }
    }

    void addTelemetry(String cap, Object val)
    {
        if(robotSettings.debug_dashboard) packet.put(cap, val);
        if(robotSettings.debug_telemetry) telemetry.addData(cap, val);
    }

    void sendTelemetry()
    {
        if(robotSettings.debug_dashboard) dashboard.sendTelemetryPacket(packet);
        if(robotSettings.debug_telemetry) telemetry.update();
    }

    ////////////////
    //calculations//
    ////////////////
    double findAngleError(double currentAngle, double targetAngle)
    {
        targetAngle = scaleAngle(targetAngle);
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError -= 360;
        } else if (angleError < -180) {
            angleError += 360;
        }
        return -angleError;
    }

    double scaleAngle(double angle)// scales an angle to fit in -180 to 180
    {
        if (angle > 180) { return angle - 360; }
        if (angle < -180) { return angle + 360; }
        return angle;
    }

    double getAngleFromXY(double X, double Y)
    {
        return Math.atan2(X, Y)*(180 / Math.PI);
    }

    double[] getXYFromAngle(double angle)
    {
        // deg to rad
        angle /= (180 / Math.PI);

        //rad to X,Y
        double[] XY = new double[2];
        XY[0] = Math.sin(angle);
        XY[1] = Math.cos(angle);
        double total = Math.abs(XY[0]) + Math.abs(XY[1]);
        XY[0] /= total;
        XY[1] /= total;

        return XY;
    }

    /////////
    //other//
    /////////
    void delay(long ms){
        long last = System.currentTimeMillis();
        while(System.currentTimeMillis() - last < ms)
        {
            if(stop())break;
        }
    }

    void sleep(long ms)
    {
        try {
            currentThread().sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    boolean stop() { return emergencyStop || gamepad1.back || gamepad2.back || opMode.isStopRequested(); }
}

class RobotUsage
{
    boolean useDrive, useComplexMovement, useLauncher, useGrabber;
    PositionUsage positionUsage;
    VisionUsage visionUsage;

    RobotUsage()
    {
        setAllToValue(true);
    }
    RobotUsage(boolean value)
    {
        setAllToValue(value);
    }
    RobotUsage(boolean useDrive, boolean useComplexMovement, boolean useLauncher, boolean useGrabber, PositionUsage positionUsage, VisionUsage visionUsage)
    {
        this.useDrive = useDrive;
        this.useComplexMovement = useComplexMovement;
        this.useLauncher = useLauncher;
        this.useGrabber = useGrabber;
        this.positionUsage = positionUsage;
        this.visionUsage = visionUsage;
    }

    void setAllToValue(boolean value)
    {
        this.useDrive = value;
        this.useComplexMovement = value;
        this.useLauncher = value;
        this.useGrabber = value;
        this.positionUsage = new PositionUsage(value);
        this.visionUsage = new VisionUsage(value);
    }
}

class PositionUsage
{
    boolean usePosition, useEncoders, useDistanceSensors, useCamera, usePositionThread;

    PositionUsage()
    {
        setAllToValue(true);
    }
    PositionUsage(boolean value)
    {
        setAllToValue(value);
    }
    PositionUsage(boolean usePosition, boolean usePositionThread, boolean useEncoders, boolean useDistanceSensors, boolean useCamera)
    {
        this.usePosition = usePosition;
        this.usePositionThread = usePositionThread;
        this.useEncoders = useEncoders;
        this.useDistanceSensors = useDistanceSensors;
        this.useCamera = useCamera;
    }

    void setAllToValue(boolean value)
    {
        this.usePosition = value;
        this.usePositionThread = value;
        this.useEncoders = value;
        this.useDistanceSensors = value;
        this.useCamera = value;
    }

    boolean useThread(){return usePosition && usePositionThread;}
    boolean positionTrackingEnabled(){return usePosition && (useDistanceSensors || useEncoders || useCamera);}
}

class VisionUsage
{
    boolean useVision, useVuforia, useVuforiaInThread, useTensorFlow, useTensorFlowInTread, useOpenCV;

    VisionUsage(){setAllToValue(true);}
    VisionUsage(boolean value)
    {
        setAllToValue(value);
    }
    VisionUsage(boolean useVision, boolean useVuforia, boolean useVuforiaInThread, boolean useTensorFlow, boolean useTensorFlowInTread, boolean useOpenCV)
    {
        this.useVision = useVision;
        this.useVuforia = useVuforia;
        this.useVuforiaInThread = useVuforiaInThread;
        this.useOpenCV = useOpenCV;
        this.useTensorFlow = useTensorFlow;
        this.useTensorFlowInTread = useTensorFlowInTread;
    }

    void setAllToValue(boolean value)
    {
        this.useVision = value;
        this.useVuforia = value;
        this.useVuforiaInThread = value;
        this.useOpenCV = value;
        this.useTensorFlow = value;
        this.useTensorFlowInTread = value;
    }

    boolean useThread() { return useVision && useVuforia && (useVuforiaInThread || (useTensorFlow && useTensorFlowInTread)); }
}

class RobotSettings
{
    /////////////
    //user data//
    /////////////
    //debug
    protected boolean debug_telemetry = true;
    protected boolean debug_dashboard = true; // turn this to false during competition
    protected boolean debug_methods = true;

    //database
    protected String dataBaseName = "FIRST_INSPIRE_2020";

    RobotSettings(){}
}

class RobotSettingsMain
{
    protected RobotSettings robotSettings;
    protected GrabberSettings grabberSettings;
    protected LauncherSettings launcherSettings;
    protected HardwareSettings hardwareSettings;
    protected MovementSettings movementSettings;
    protected PositionSettings positionSettings;
    protected VisionSettings visionSettings;

    RobotSettingsMain()
    {
        robotSettings = new RobotSettings();
        grabberSettings = new GrabberSettings();
        launcherSettings = new LauncherSettings();
        hardwareSettings = new HardwareSettings();
        movementSettings = new MovementSettings();
        positionSettings = new PositionSettings();
        visionSettings = new VisionSettings();
    }
    RobotSettingsMain(RobotSettings robotSettings, GrabberSettings grabberSettings, LauncherSettings launcherSettings, HardwareSettings hardwareSettings, MovementSettings movementSettings, PositionSettings positionSettings, VisionSettings visionSettings)
    {
        this.robotSettings = robotSettings;
        this.grabberSettings = grabberSettings;
        this.launcherSettings = launcherSettings;
        this.hardwareSettings = hardwareSettings;
        this.movementSettings = movementSettings;
        this.positionSettings = positionSettings;
        this.visionSettings = visionSettings;
    }
}