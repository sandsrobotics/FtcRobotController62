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

/**
 * Main class that contains all robot parts, settings, and other objects for
 * robot functionality. Allows all parts to talk to eachother and provides high
 * level functions and math.
 */
@Config
public class Robot {
    ///////////////////
    // other variables//
    ///////////////////
    public RobotHardware robotHardware;
    public Movement movement;
    public Vision vision;
    public Launcher launcher;
    public ComplexMovement complexMovement;
    public PositionTracker positionTracker;
    public Grabber grabber;

    // objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected BNO055IMU imu;
    protected LinearOpMode opMode;
    protected AppDatabase db;
    protected RobotUsage robotUsage;
    protected RobotSettings robotSettings;

    // other
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    TelemetryPacket packet = new TelemetryPacket();
    public static boolean emergencyStop = false;

    /**
     * constructor for main robot class with custom settings and usages
     * 
     * @param opMode            object passed in from main class containing
     *                          hardwareMap, gamepads, telemetry, and other
     *                          objects/funtions
     * @param robotUsage        contains all the flags to turn on and off all the
     *                          parts of the robot
     * @param robotSettingsMain contains all the settings for the diffrent parts of
     *                          the robot
     */
    Robot(LinearOpMode opMode, RobotUsage robotUsage, RobotSettingsMain robotSettingsMain) {
        init(opMode, robotUsage, robotSettingsMain);
    }

    /**
     * constructor for main robot class with default settings and custom usages
     * 
     * @param opMode            object passed in from main class containing
     *                          hardwareMap, gamepads, telemetry, and other
     *                          objects/funtions
     * @param robotUsage        contains all the flags to turn on and off all the
     *                          parts of the robot
     * @param robotSettingsMain contains all the settings for the diffrent parts of
     *                          the robot
     */
    Robot(LinearOpMode opMode, RobotUsage robotUsage) {
        init(opMode, robotUsage, new RobotSettingsMain());
    }

    /**
     * constructor for main robot class with default settings and usages (default
     * usage = all flags true)
     * 
     * @param opMode            object passed in from main class containing
     *                          hardwareMap, gamepads, telemetry, and other
     *                          objects/funtions
     * @param robotUsage        contains all the flags to turn on and off all the
     *                          parts of the robot
     * @param robotSettingsMain contains all the settings for the diffrent parts of
     *                          the robot
     */
    Robot(LinearOpMode opMode) {
        init(opMode, new RobotUsage(), new RobotSettingsMain());
    }

    /**
     * sets and initializes the variables and hardware in the robot class and all
     * its components
     * 
     * @param opMode            object passed in from main class containing
     *                          hardwareMap, gamepads, telemetry, and other
     *                          objects/funtions
     * @param robotUsage        contains all the flags to turn on and off all the
     *                          parts of the robot
     * @param robotSettingsMain contains all the settings for the diffrent parts of
     *                          the robot
     */
    private void init(LinearOpMode opMode, RobotUsage robotUsage, RobotSettingsMain robotSettingsMain) {
        this.robotUsage = robotUsage;
        this.robotSettings = robotSettingsMain.robotSettings;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        robotHardware = new RobotHardware(this, robotSettingsMain.hardwareSettings);

        if (robotUsage.positionUsage.usePosition)
            positionTracker = new PositionTracker(this, robotSettingsMain.positionSettings);
        if (robotUsage.useDrive)
            movement = new Movement(this, robotSettingsMain.movementSettings);
        if (robotUsage.visionUsage.useVision)
            vision = new Vision(this, robotSettingsMain.visionSettings);
        if (robotUsage.useLauncher)
            launcher = new Launcher(this, robotSettingsMain.launcherSettings);
        if (robotUsage.useComplexMovement)
            complexMovement = new ComplexMovement(this);
        if (robotUsage.useGrabber) {
            grabber = new Grabber(this, robotSettingsMain.grabberSettings);
            addTelemetry("grabber", " init");
        }

        initHardware();
        if (robotUsage.useDrive || (robotUsage.positionUsage.usePosition && robotUsage.positionUsage.useEncoders)
                || robotUsage.useComplexMovement)
            robotHardware.initDriveMotors();
        if (robotUsage.useLauncher)
            robotHardware.initLauncherMotors();
        if (robotUsage.visionUsage.useVision)
            vision.initAll();
        if (robotUsage.useGrabber)
            robotHardware.initGrabberHardware();
        if (robotUsage.positionUsage.usePosition && robotUsage.positionUsage.useDistanceSensors) {
            robotHardware.initUltrasonicSensors();
        }
    }

    /**
     * initializes sensors, telemetry, and database contained in the robot class
     */
    void initHardware() {
        ///////////
        // sensors//
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

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            delay(50);
            opMode.idle();
        }

        /////////////
        // dashboard//
        /////////////
        if (robotSettings.debug_dashboard)
            dashboard = FtcDashboard.getInstance();
        startTelemetry();

        /////////////
        // data base//
        /////////////
        db = Room.databaseBuilder(AppUtil.getDefContext(), AppDatabase.class, robotSettings.dataBaseName).build();
    }

    // ------------------My Methods------------------//

    /**
     * starts/initializes telemetry and robot parts
     * 
     * @param resetGrabberPos flag to reset position of grabber
     * @param isAuto          flag to tell the robot class and its parts whether it
     *                        is an autonomous or not
     */
    void start(boolean resetGrabberPos, boolean isAuto) {
        startTelemetry();
        if (robotUsage.positionUsage.useThread())
            positionTracker.start();
        if (robotUsage.visionUsage.useThread())
            vision.start();

        if (robotUsage.useGrabber && resetGrabberPos)
            grabber.initGrabberPos();
        if (!isAuto && robotUsage.useLauncher)
            launcher.initFrogLegs();
    }

    /////////////
    // telemetry//
    /////////////

    /**
     * if sending telemetry to dashboard it starts the dashboard telemetry by
     * creating a new telemetry packet object
     */
    void startTelemetry() {
        if (robotSettings.debug_dashboard) {
            packet = new TelemetryPacket();
        }
    }

    /**
     * adds telemetry to both dashboard and app based on flags in robotSettings
     * 
     * @param cap the string that defines what the value is representing
     * @param val the value that comes after the caption, seperated by a :
     */
    void addTelemetry(String cap, Object val) {
        if (robotSettings.debug_dashboard)
            packet.put(cap, val);
        if (robotSettings.debug_telemetry)
            telemetry.addData(cap, val);
    }

    /**
     * sends the telemetry to the dashbaord and app based on flags in robotSettings
     */
    void sendTelemetry() {
        if (robotSettings.debug_dashboard)
            dashboard.sendTelemetryPacket(packet);
        if (robotSettings.debug_telemetry)
            telemetry.update();
    }

    ////////////////
    // calculations//
    ////////////////

    /**
     * find the smallest angle between the target and current angle
     * 
     * @param currentAngle the current angle you are at
     * @param targetAngle  the angle that you want to get to
     * @return the angle you need to turn from currentAngle to get to targetAngle
     */
    double findAngleError(double currentAngle, double targetAngle) {
        targetAngle = scaleAngle(targetAngle);
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError -= 360;
        } else if (angleError < -180) {
            angleError += 360;
        }
        return -angleError;
    }

    /**
     * scales angle
     * 
     * @param angle input angle from 0 - 360
     * @return scaled angle from -180 - 180
     */
    double scaleAngle(double angle) {
        if (angle > 180) {
            return angle - 360;
        }
        if (angle < -180) {
            return angle + 360;
        }
        return angle;
    }

    /**
     * gets the angle of a line based on its x and y end point
     * 
     * @param X the x length
     * @param Y the y length
     * @return the angle of the line
     */
    double getAngleFromXY(double X, double Y) {
        return Math.atan2(X, Y) * (180 / Math.PI);
    }

    /**
     * gets x and y distances based on angle
     * 
     * @param angle angle to convert to x and y
     * @return an array with x and y (x + y = 1)
     */
    double[] getXYFromAngle(double angle) {
        // deg to rad
        angle /= (180 / Math.PI);

        // rad to X,Y
        double[] XY = new double[2];
        XY[0] = Math.sin(angle);
        XY[1] = Math.cos(angle);
        double total = Math.abs(XY[0]) + Math.abs(XY[1]);
        XY[0] /= total;
        XY[1] /= total;

        return XY;
    }

    /////////
    // other//
    /////////

    /**
     * runs the thread waiting for the delay to be up - allows for delay to be
     * inturupted with flags in Robot.stop()
     * 
     * @param ms the miliseconds that you want the thread to sleep
     */
    void delay(long ms) {
        long last = System.currentTimeMillis();
        while (System.currentTimeMillis() - last < ms) {
            if (stop())
                break;
        }
    }

    /**
     * puts the thread that calls this method to sleep for a certin time
     * 
     * @param ms the miliseconds that you want the thread to sleep
     */
    void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    boolean stop() {
        return emergencyStop || gamepad1.back || gamepad2.back || opMode.isStopRequested();
    }
}

/**
 * contains flags to turn on and off all the parts of the robot
 */
class RobotUsage {
    boolean useDrive, useComplexMovement, useLauncher, useGrabber;
    PositionUsage positionUsage;
    VisionUsage visionUsage;

    /**
     * sets all flags in RobotUsage to true
     */
    RobotUsage() {
        setAllToValue(true);
    }

    /**
     * sets all flags in RobotUsage to the variable value
     * 
     * @param value sets all the flags in RobotUsage to value
     */
    RobotUsage(boolean value) {
        setAllToValue(value);
    }

    /**
     * sets all values in RobotUsage based on there corrisponding variables
     * 
     * @param useDrive           controls the wheel motors and encoders and
     *                           initilizes the movemnt classes
     * @param useComplexMovement initilizes the ComplexMovement class that can
     *                           record and replay movements plus store them in a
     *                           database - requires useDrive to be enabled
     * @param useLauncher        contorls the launcher and intake - if you want to
     *                           use the auto-launch functions you also need to
     *                           enable useDrive and position tracking from
     *                           positionUsage
     * @param useGrabber         contorls the grabber part - if you want to use the
     *                           auto-drop function(expirimental) you also need to
     *                           enable useDrive and position tracking from
     *                           positionUsage
     * @param positionUsage      controls all the flags related to position tracking
     * @param visionUsage        controls all the flags related to vision and camera
     */
    RobotUsage(boolean useDrive, boolean useComplexMovement, boolean useLauncher, boolean useGrabber,
            PositionUsage positionUsage, VisionUsage visionUsage) {
        this.useDrive = useDrive;
        this.useComplexMovement = useComplexMovement;
        this.useLauncher = useLauncher;
        this.useGrabber = useGrabber;
        this.positionUsage = positionUsage;
        this.visionUsage = visionUsage;
    }

    /**
     * sets all flags in RobotUsage to value
     * 
     * @param value sets all the variables in RobotUsage to value
     */
    void setAllToValue(boolean value) {
        this.useDrive = value;
        this.useComplexMovement = value;
        this.useLauncher = value;
        this.useGrabber = value;
        this.positionUsage = new PositionUsage(value);
        this.visionUsage = new VisionUsage(value);
    }
}

/**
 * contains all flags related to tracking position
 */
class PositionUsage {
    boolean usePosition, useEncoders, useDistanceSensors, useCamera, usePositionThread;

    /**
     * sets all flags in PositionUsage to true
     */
    PositionUsage() {
        setAllToValue(true);
    }

    /**
     * sets all flags in RobotUsage to the variable value
     * 
     * @param value sets all the flags in RobotUsage
     */
    PositionUsage(boolean value) {
        setAllToValue(value);
    }

    /**
     * sets all flags based on there corresponding variables
     * 
     * @param usePosition        this flag turns on and off position tracking
     * @param usePositionThread  this flag runs position tracking in a thread so you
     *                           dont have to constantly call the tracking methods
     * @param useEncoders        this flag runs the position tracking using the
     *                           drive wheel encoders
     * @param useDistanceSensors this flag runs the position tracking using the
     *                           distance sensors
     * @param useCamera          this flag runs the position tracking using the
     *                           intel camera
     */
    PositionUsage(boolean usePosition, boolean usePositionThread, boolean useEncoders, boolean useDistanceSensors,
            boolean useCamera) {
        this.usePosition = usePosition;
        this.usePositionThread = usePositionThread;
        this.useEncoders = useEncoders;
        this.useDistanceSensors = useDistanceSensors;
        this.useCamera = useCamera;
    }

    /**
     * sets all flags to the variable value
     * 
     * @param value the value you want to set all the flags to
     */
    void setAllToValue(boolean value) {
        this.usePosition = value;
        this.usePositionThread = value;
        this.useEncoders = value;
        this.useDistanceSensors = value;
        this.useCamera = value;
    }

    /**
     * tells whether or not a thread is used for position tracking
     * 
     * @return if a thread is being used for position tracking
     */
    boolean useThread() {
        return usePosition && usePositionThread;
    }

    /**
     * tells whether or not the position is being tracked
     * 
     * @return if position is being tracked
     */
    boolean positionTrackingEnabled() {
        return usePosition && (useDistanceSensors || useEncoders || useCamera);
    }
}

/**
 * contains all flags related to vision
 */
class VisionUsage {
    boolean useVision, useVuforia, useVuforiaInThread, useTensorFlow, useTensorFlowInTread, useOpenCV;

    /**
     * sets all flags in VisionUsage to true
     */
    VisionUsage() {
        setAllToValue(true);
    }

    /**
     * sets all flags in VisionUsage to the variable value
     * 
     * @param value sets all the flags in VisionUsage
     */
    VisionUsage(boolean value) {
        setAllToValue(value);
    }

    /**
     * sets all flags based on there corresponding variables
     * 
     * @param useVision            this flag makes the Vision class
     * @param useVuforia           this flag truns on and initializes vuforia
     * @param useVuforiaInThread   this flag runs vuforia in a thread so you can
     *                             constantly check for viewmarks - requires
     *                             useVuforia to be true
     * @param useTensorFlow        this flag truns on and initializes tensorflow -
     *                             requires vuforia to be on
     * @param useTensorFlowInTread this flag runs tensorflow in a thread - requires
     *                             useVuforia and useTensorFlow to be true
     * @param useOpenCV            this flag initilizes and starts the opencv
     *                             pipeline
     */
    VisionUsage(boolean useVision, boolean useVuforia, boolean useVuforiaInThread, boolean useTensorFlow,
            boolean useTensorFlowInTread, boolean useOpenCV) {
        this.useVision = useVision;
        this.useVuforia = useVuforia;
        this.useVuforiaInThread = useVuforiaInThread;
        this.useOpenCV = useOpenCV;
        this.useTensorFlow = useTensorFlow;
        this.useTensorFlowInTread = useTensorFlowInTread;
    }

    /**
     * sets all flags to the variable value
     * 
     * @param value the value that all flags are set to
     */
    void setAllToValue(boolean value) {
        this.useVision = value;
        this.useVuforia = value;
        this.useVuforiaInThread = value;
        this.useOpenCV = value;
        this.useTensorFlow = value;
        this.useTensorFlowInTread = value;
    }

    /**
     * checks whether or not the Vision class needs/uses a thread
     * 
     * @return if a thread was being used for vision
     */
    boolean useThread() {
        return useVision && useVuforia && (useVuforiaInThread || (useTensorFlow && useTensorFlowInTread));
    }
}

/**
 * stores settings for the Robot class
 */
class RobotSettings {
    /////////////
    // user data//
    /////////////
    // debug
    protected boolean debug_telemetry = true;
    protected boolean debug_dashboard = true; // turn this to false during competition
    protected boolean debug_methods = true;

    // database
    protected String dataBaseName = "FIRST_INSPIRE_2020";

    /**
     * normal constructor - sets all values to preset values
     */
    RobotSettings() {
    }
}

/**
 * stores the settings for all parts/classes of the robot
 */
class RobotSettingsMain {
    protected RobotSettings robotSettings;
    protected GrabberSettings grabberSettings;
    protected LauncherSettings launcherSettings;
    protected HardwareSettings hardwareSettings;
    protected MovementSettings movementSettings;
    protected PositionSettings positionSettings;
    protected VisionSettings visionSettings;

    /**
     * sets all setting classes to default preset values
     */
    RobotSettingsMain() {
        robotSettings = new RobotSettings();
        grabberSettings = new GrabberSettings();
        launcherSettings = new LauncherSettings();
        hardwareSettings = new HardwareSettings();
        movementSettings = new MovementSettings();
        positionSettings = new PositionSettings();
        visionSettings = new VisionSettings();
    }

    /**
     * sets all setting classes to custom values from variables
     * 
     * @param robotSettings    settings for Robot class
     * @param grabberSettings  settings for Grabber class
     * @param launcherSettings settings for Launcher class
     * @param hardwareSettings settings for RobotHardware class
     * @param movementSettings settings for Movement class
     * @param positionSettings settings for Position class
     * @param visionSettings   settings for Vision class
     */
    RobotSettingsMain(RobotSettings robotSettings, GrabberSettings grabberSettings, LauncherSettings launcherSettings,
            HardwareSettings hardwareSettings, MovementSettings movementSettings, PositionSettings positionSettings,
            VisionSettings visionSettings) {
        this.robotSettings = robotSettings;
        this.grabberSettings = grabberSettings;
        this.launcherSettings = launcherSettings;
        this.hardwareSettings = hardwareSettings;
        this.movementSettings = movementSettings;
        this.positionSettings = positionSettings;
        this.visionSettings = visionSettings;
    }
}