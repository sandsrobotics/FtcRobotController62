package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Config
public class Vision extends Thread
{
    ////////////////////
    // other variables//
    ////////////////////
    // object location
    protected volatile OpenGLMatrix[] lastTrackablesLocations = new OpenGLMatrix[5]; //stores the last known positionTracker of trackables
    protected volatile OpenGLMatrix[] currentTrackablesLocations = new OpenGLMatrix[5]; //stores the current positionTracker of trackables if they are visible
    protected volatile OpenGLMatrix lastCalculatedRobotLocation = new OpenGLMatrix(); //stores the last known robot location from trackables
    protected volatile OpenGLMatrix currentCalculatedRobotLocation = new OpenGLMatrix(); //stores the current robot location if any trackables are visible
    protected volatile boolean anyTrackableFound = false; //stores whether an trackables are visible

    // some vuforia stuff
    protected VuforiaLocalizer vuforia = null; //API to setup trackables and Tfod
    protected VuforiaTrackables trackables; //object that check if any trackables are visible
    protected VuforiaLocalizer.Parameters parameters; //parameters to setup vuforia

    //some openCV stuff
    protected OpenCvWebcam webcam; //a webcam object to get image
    protected OpenCvCamera phoneCam; //a phone camera to get picture from phone
    protected SkystoneDeterminationPipeline pipeline; //pipeline to run OpenCV ring detection

    //some tensorFlow stuff
    TFObjectDetector tfod; //a model object to find any rings
    protected volatile List<Recognition> tfodCurrentRecognitions; //stores the current positionTracker and amount of rings if available
    protected volatile boolean anyTfodObjectsFound = false; //stores whether any objects where found

    //other
    protected int cameraMonitorViewId;
    protected boolean usingWebcam; // weather or not you are using a web-cam or phone

    //other class
    Robot robot; //to use values/methods from the robot object, other objects, and LinearOpMode
    VisionSettings visionSettings; //just an object to store all user set variables for vision


    //////////////////
    //Vision Methods//
    //////////////////
    Vision(Robot robot)
    {
        visionSettings = new VisionSettings();
        this.robot = robot;
    }
    Vision(Robot robot, VisionSettings visionSettings)
    {
        this.visionSettings = visionSettings;
        this.robot = robot;
    }

    void initAll(boolean useVuforia, boolean useOpenCV, boolean useTensorFlow) //sets up all the vision objects and camera
    {
        initCamera();
        checkCameraType();

        if(useVuforia)
        {
            initVuforia();
            loadAsset("UltimateGoal");
            setAllTrackablesNames();
            setAllTrackablesPosition();
            setPhoneTransform(visionSettings.phonePosition, visionSettings.phoneRotation);

            if(useTensorFlow)
            {
                initTfod();
            }
        }

        if(useOpenCV)
        {
            initOpenCV();
        }
    }

    void initAll() //the method above but it takes values from RobotUsage
    {
        initAll(robot.robotUsage.visionUsage.useVuforia, robot.robotUsage.visionUsage.useOpenCV, robot.robotUsage.visionUsage.useTensorFlow);
        robot.robotHardware.initVisionHardware();
        robot.robotHardware.mainCamServo.setPosition(visionSettings.camServoStartPos);

    }

    void initCamera() //gets the camera Id to make initialization easier for vision objects
    {
        cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
    }

    void checkCameraType()
    {
        try
        {
            robot.hardwareMap.get(WebcamName.class, "Webcam 1");
            usingWebcam = true;
        }
        catch(Exception e)
        {
            usingWebcam = false;
        }
    }

    ///////////////////
    //Vuforia Methods//
    ///////////////////
    void initVuforia() //initializes vuforia
    {
        //make a parameters object
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //define the parameters object
        parameters.vuforiaLicenseKey = visionSettings.VUFORIA_KEY;
        parameters.cameraDirection = visionSettings.CAMERA_CHOICE_V;
        parameters.useExtendedTracking = visionSettings.useExtendedTracking;
        if(robot.robotUsage.visionUsage.useTensorFlow && usingWebcam) parameters.cameraName = robot.hardwareMap.get(WebcamName.class, "Webcam 1");

        //Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void startDashboardCameraStream(int maxFps, boolean useVuforia)
    {
        if(useVuforia && vuforia != null) FtcDashboard.getInstance().startCameraStream(vuforia,maxFps);
        else if(tfod != null) FtcDashboard.getInstance().startCameraStream(tfod,maxFps);
    } // starts a dashboard stream at a certain fps
    void stopDashboardCameraStream(){FtcDashboard.getInstance().stopCameraStream();} // stops the dashboard stream

    void loadAsset(String assetName) { trackables = vuforia.loadTrackablesFromAsset(assetName); } // loads the vuforia assets from a file

    void setAllTrackablesNames() // sets vuforia tackable names
    {

        setTrackableName(1,"Red Tower Goal Target");
        setTrackableName(2,"Red Alliance Target");
        setTrackableName(0,"Blue Tower Goal Target");
        setTrackableName(3,"Blue Alliance Target");
        setTrackableName(4,"Front Wall Target");
    }

    void setAllTrackablesPosition() // sets vuforia tackable positions
    {
        setTrackableTransform(2, new float[]{0, 0, 0}, new float[]{0, 0, 0});
        setTrackableTransform(3, new float[]{0, 0, 0}, new float[]{0, 0, 0});
        setTrackableTransform(4, new float[]{0, 0, 0}, new float[]{0, 0, 0});
        setTrackableTransform(0, new float[]{0, 0, 0}, new float[]{0, 0, 0});
        setTrackableTransform(1, new float[]{0, 0, 0}, new float[]{0, 0, 0});
    }

    void setTrackableName(int posInTrackables, String name) //sets the name for a single trackable
    {
        trackables.get(posInTrackables).setName(name);
    }
    
    void setTrackableTransform(int posInTrackables, float[] position, float[] angles) //sets the positionTracker for a single trackable // positionTracker is in inches and rotation is in deg with order XYZ
    {
        for(int i = 0; i < position.length; i++) position[i] *= Constants.mmPerInch;

        trackables.get(posInTrackables).setLocation(OpenGLMatrix
                .translation(position[0], position[1], position[2])
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, angles[0], angles[1], angles[2])));
    }

    void setPhoneTransform(float[] position, float[] angles) //sets the phone transform // positions is in INCHES: X is INCHES left from center line, Y is INCHES above ground, Z is INCHES forward from center line. rotation is in order XYZ in deg
    {
        for(int i = 0; i < position.length; i++) position[i] *= Constants.mmPerInch;

        if (visionSettings.CAMERA_CHOICE_V == BACK)
        {
            angles[1] -= 90;
        }
        else
        {
            angles[1] += 90;
        }

        if (visionSettings.PHONE_IS_PORTRAIT)
        {
            angles[0] += 90;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(position[2], position[0], position[1])
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, angles[1], angles[2], angles[0]));

        for (VuforiaTrackable trackable : trackables)
        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    void setPhoneTorch(boolean on) { CameraDevice.getInstance().setFlashTorchMode(on); } //turns on/off the phone flashlight

    void activateVuforia(){trackables.activate();} // activates vuforia trackables to find objects
    void deactivateVuforia(){trackables.deactivate();} // deactivates vuforia trackables if not needed

    void findAllTrackables() // will find all objects(pictures on wall) that are visible
    {
        int i = 0;
        anyTrackableFound = false;

        for(VuforiaTrackable t:trackables)
        {
            if (((VuforiaTrackableDefaultListener) t.getListener()).isVisible())
            {
                anyTrackableFound = true;

                currentTrackablesLocations[i] = ((VuforiaTrackableDefaultListener) t.getListener()).getFtcCameraFromTarget();
                lastTrackablesLocations[i] = currentTrackablesLocations[i];

                OpenGLMatrix robotPos = ((VuforiaTrackableDefaultListener) t.getListener()).getUpdatedRobotLocation();

                if (robotPos != null)
                {
                    currentCalculatedRobotLocation = robotPos;
                    lastCalculatedRobotLocation = currentCalculatedRobotLocation;
                }
            }
            else currentTrackablesLocations[i] = null;
            i++;
        }

        if(!anyTrackableFound) currentCalculatedRobotLocation = null;
    }

    OpenGLMatrix getCurrentGaolLocation() //returns the current positionTracker for goal picture if visible
    {
        return currentTrackablesLocations[visionSettings.goalPictureNum];
    }

    Orientation getTrackableAngles(OpenGLMatrix m)
    {
        if(m != null) return Orientation.getOrientation(m, EXTRINSIC, XYZ, DEGREES);
        return null;
    }

    //////////////////////
    //tensorFlow Methods//
    //////////////////////
    void initTfod()
    {
        int tfodMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = visionSettings.minResultConfidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(visionSettings.TFOD_MODEL_ASSET, visionSettings.LABEL_FIRST_ELEMENT, visionSettings.LABEL_SECOND_ELEMENT);
    }

    void activateTfod(){tfod.activate();}
    void deactivateTfod(){tfod.shutdown();}

    void tofdActivationSequence()
    {
        activateTfod();
        tfod.setZoom(2, (double)16/(double)9);
    }

    Recognition getHighestConfidence()
    {
        Recognition out = null;
        if(anyTfodObjectsFound)
        {
            for(Recognition rec: tfodCurrentRecognitions)
            {
                if(out == null) out = rec;
                else if(rec.getConfidence() > out.getConfidence()) out = rec;
            }
        }
        return out;
    }

    int getNumOfRings(Recognition rec)
    {
        if(rec != null)
        {
            if(rec.getLabel().equals("QUAD")) return 4;
            else if(rec.getLabel().equals("SINGLE")) return 1;
        }
        return 0;
    }
    int getNumOfRings() { return getNumOfRings(getHighestConfidence()); }

    void findAllTfodObjects()
    {
        tfodCurrentRecognitions = tfod.getRecognitions();
        anyTfodObjectsFound = (tfodCurrentRecognitions != null);
    }

    int runTfodSequenceForRings()
    {
        findAllTfodObjects();
        return getNumOfRings();
    }

    //////////////////
    //OpenCV Methods//
    //////////////////
    void initOpenCV()
    {
        if(usingWebcam)
        {
            //creating a camera object
            webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            //creating a openCV pipeline
            pipeline = new Vision.SkystoneDeterminationPipeline();

            //integrate the openCV pipeline with the camera
            webcam.setPipeline(pipeline);

            //start camera
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                }
            });
        }
        else
        {
            //creating a camera object
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(visionSettings.CAMERA_CHOICE_O, cameraMonitorViewId);

            //creating a openCV pipeline
            pipeline = new Vision.SkystoneDeterminationPipeline();

            //integrate the openCV pipeline with the camera
            phoneCam.setPipeline(pipeline);
            phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

            //start camera
            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                }
            });
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone positionTracker
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

/*
    @Config
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        //////////////////
        //user variables//
        //////////////////
        //for ring detection
        public static Point RING_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        public static int RING_REGION_WIDTH = 35;
        public static int RING_REGION_HEIGHT = 25;

        public static int FOUR_RING_THRESHOLD = 150;
        public static int ONE_RING_THRESHOLD = 135;

        //for other
        public static Point OTHER_TOPLEFT_ANCHOR_POINT = new Point(60,70);

        public static int OTHER_REGION_WIDTH = 200;
        public static int OTHER_REGION_HEIGHT = 100;

        //these values are in the HSV color space(openCV uses 0-180 for H, and 0-255 for S and V)
        protected int[] OTHER_COLOR_UPPER = new int[]{60,255,255};
        protected int[] OTHER_COLOR_LOWER = new int[]{20,100,50};

        public static int OTHER_THRESHOLD = 50;

        ///////////////////
        //other variables//
        ///////////////////
        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        //edge points for the box for scanning for rings
        Point Ring_region1_pointA = new Point(
                RING_TOPLEFT_ANCHOR_POINT.x,
                RING_TOPLEFT_ANCHOR_POINT.y);
        Point Ring_region1_pointB = new Point(
                RING_TOPLEFT_ANCHOR_POINT.x + RING_REGION_WIDTH,
                RING_TOPLEFT_ANCHOR_POINT.y + RING_REGION_HEIGHT);

        public static Point Other_region1_pointA = OTHER_TOPLEFT_ANCHOR_POINT;
        public static Point Other_region1_pointB = new Point(
                OTHER_TOPLEFT_ANCHOR_POINT.x + OTHER_REGION_WIDTH,
                OTHER_TOPLEFT_ANCHOR_POINT.y + OTHER_REGION_HEIGHT);



        //images
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat imgCopy;

        int avg1;
        int positionTracker;

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        boolean rectInImg(Mat img, Rect rect)
        {
            return  rect.x >= 0 &&
                    rect.y >= 0 &&
                    rect.x + rect.width <= img.cols() &&
                    rect.y + rect.height <= img.rows();
        }

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(Ring_region1_pointA, Ring_region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            imgCopy = input.clone();
            inputToCb(imgCopy);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    Ring_region1_pointA, // First point which defines the rectangle
                    Ring_region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    Other_region1_pointA, // First point which defines the rectangle
                    Other_region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                positionTracker = 4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                positionTracker = 1;
            }else{
                positionTracker = 0;
            }

            contours = getColorRangeContoursFromImage(imgCopy, OTHER_COLOR_LOWER, OTHER_COLOR_UPPER, Other_region1_pointA, Other_region1_pointB);

            for(int i = 0; i < contours.size(); i++)
            {
                if(Imgproc.contourArea(contours.get(i)) > OTHER_THRESHOLD){Imgproc.drawContours(input, contours, i, GREEN, 2);}
            }

            return input;
        }

        public List<MatOfPoint> getColorRangeContoursFromImage(Mat input, int[] lower, int[] upper, Point upperLeftPoint, Point lowerRightPoint)
        {
            //prepossessing
            Mat process = input;

            Imgproc.cvtColor(process,process,Imgproc.COLOR_RGB2HSV);
            Core.inMeasuringRange(process, new Scalar(lower[0], lower[1], lower[2], 0), new Scalar(upper[0], upper[1], upper[2], 0), process);

            Rect rect = new Rect(upperLeftPoint, new Size(50,50));

            if(rectInImg(process, rect)) process = process.submat(rect);

            //finding contours
            List<MatOfPoint> out = new ArrayList<>();
            Imgproc.findContours(process, out, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            return out;
        }
    }

 */

    /////////////////
    //vision thread//
    /////////////////
    public void run()
    {
        if(robot.robotUsage.visionUsage.useVuforiaInThread)activateVuforia();
        if(robot.robotUsage.visionUsage.useTensorFlow && robot.robotUsage.visionUsage.useTensorFlowInTread) tofdActivationSequence();

        while(!this.isInterrupted() && !robot.opMode.isStopRequested())
        {
            if(robot.robotUsage.visionUsage.useVuforiaInThread)findAllTrackables();
            if(robot.robotUsage.visionUsage.useTensorFlow && robot.robotUsage.visionUsage.useTensorFlowInTread) findAllTfodObjects();
        }

        if(robot.robotUsage.visionUsage.useVuforiaInThread)deactivateVuforia();
        if(robot.robotUsage.visionUsage.useTensorFlow && robot.robotUsage.visionUsage.useTensorFlowInTread) deactivateTfod();
    }

    public void stopThread()
    {
        this.interrupt();
    }
}

class VisionSettings
{
    //////////////////
    //user variables//
    //////////////////
    //just some stuff to get a working vuforia object
    protected final VuforiaLocalizer.CameraDirection CAMERA_CHOICE_V = BACK; // if you are using a phone which camera do you want to use
    protected final boolean PHONE_IS_PORTRAIT = false; // if you are using a phone which orientation is it
    protected final boolean useExtendedTracking = false;
    protected final String VUFORIA_KEY = "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";

    //to know where the phone or camera is IN INCHES!!! and degrees
    protected final float[] phonePosition = {0,0,0}; // the phone positionTracker from center of robot
    protected final float[] phoneRotation = {0,0,0}; // the phone rotation

    //to see where goal is
    protected final int goalPictureNum = 3; // which picture is the one under the goal

    //to set up easy openCV camera
    protected final OpenCvInternalCamera.CameraDirection CAMERA_CHOICE_O = OpenCvInternalCamera.CameraDirection.BACK; // if you are using a phone which camera do you want to use

    //tensorFlow
    protected final String TFOD_MODEL_ASSET = "UltimateGoal.tflite"; //what is the name of the model
    protected final String LABEL_FIRST_ELEMENT = "QUAD";
    protected final String LABEL_SECOND_ELEMENT = "SINGLE";
    protected final float minResultConfidence = .5f; //how confident does the model have to be to say there is a ring

    //main cam servo
    double camServoStartPos = .6;

    VisionSettings(){}
}