package org.firstinspires.ftc.teamcode.robot2020;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class PositionTracker extends Thread
{
    ///////////////////
    //other variables//
    ///////////////////
    //robot positionTracker
    protected volatile Position currentPosition; //positionTracker is based on the canter front of the goal to the center of the robot
    volatile boolean isInitialized = false;

    //wheels
    private int[] lastMotorPos;
    private int[] currMotorPos;

    //rotation
    volatile Orientation currentAllAxisRotations = new Orientation();
    protected double rotationOffset;

    //angular velocity
    volatile AngularVelocity currentAngularVelocity = new AngularVelocity();

    //encoders
    public volatile Position encoderPosition = new Position();

    //distance sensor positionTracker
    private long lastSensorReadingTime = System.currentTimeMillis();
    private int inMeasuringRange = -2;
    public volatile Position distSensorPosition = new Position();

    //camera
    static T265Camera slamra = null;
    private Position cameraOffset = new Position();
    public volatile Position cameraPosition = new Position();

    //other
    String fileName = "angleData";
    boolean drawDashboardField = true;

    //other class
    Robot robot;
    PositionSettings positionSettings;

    PositionTracker(Robot robot)
    {
        positionSettings = new PositionSettings();
        this.robot = robot;
        if(robot.robotUsage.positionUsage.usePositionCamera) initCam();
    }
    PositionTracker(Robot robot, PositionSettings positionSettings)
    {
        this.positionSettings = positionSettings;
        this.robot = robot;
        if(robot.robotUsage.positionUsage.usePositionCamera) initCam();
    }

    //////////
    //angles//
    //////////
    Orientation updateAngles()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angles.thirdAngle *= -1;
        angles.thirdAngle -= rotationOffset;
        angles.thirdAngle = (float)robot.scaleAngle(angles.thirdAngle);
        return angles;
    }

    void resetAngle()
    {
        rotationOffset += currentPosition.R;
    }

    ///////////////////////////
    //positionTracker finding//
    ///////////////////////////
    void getPosFromEncoder()
    {
        //get difference
        lastMotorPos = currMotorPos;
        currMotorPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
        int[] diff = new int[4];
        for(int i = 0; i < 4; i++)
        {
            diff[i] = currMotorPos[i] - lastMotorPos[i];
        }

        //get movement
        double YMove = (.25 * (diff[0] + diff[2] + diff[1] + diff[3]))/positionSettings.ticksPerInchForward;
        double XMove = (.25 * (-diff[0] + diff[2] + diff[1] - diff[3]))/positionSettings.ticksPerInchSideways;

        //rotate and add to robot positionTracker
        encoderPosition.X += YMove * Math.sin(currentPosition.R * Math.PI / 180) - XMove * Math.cos(currentPosition.R * Math.PI / 180);
        encoderPosition.Y += XMove * Math.sin(currentPosition.R * Math.PI / 180) + YMove * Math.cos(currentPosition.R * Math.PI / 180);
        encoderPosition.R = currentPosition.R;
    }

    ///////////////////
    //distance sensor//
    ///////////////////

    int isRobotInRotationRange()//checks if the current angle is close enough to one of the 90 degree increments
    {
        if(robot.robotUsage.positionUsage.useDistanceSensors) {
            for (int i = -1; i < 3; i++)
                if (Math.abs(currentPosition.R - (i * 90)) <= positionSettings.angleTolerance)
                    return i;
        }
        return -2;
    }

    private double distanceFromClosestIncrement() { return Math.abs(currentPosition.R - (inMeasuringRange * 90)); }

    private void updatePosWithDistanceSensor(boolean useCorrection)
    {
        if(inMeasuringRange > -2) {
            sleepTillNextSensorReading();
            float[] temp = robot.robotHardware.getDistancesAfterMeasure(robot.robotHardware.distSensors);

            int arrayPos = inMeasuringRange;
            if (inMeasuringRange == -1) arrayPos = 3;

            if (positionSettings.sensorPosition[arrayPos] == SensorNum.TWO) {
                float val = temp[0];
                temp[0] = temp[1];
                temp[1] = val;
            }

            double[] calcDis = new double[2];
            for (int b = 0; b < 2; b++) //does the math for both x and y axis
            {
                double dis = positionSettings.distancesFromWall[arrayPos][b];
                if (positionSettings.operations[arrayPos][b] == MathSign.ADD)
                    dis += temp[b];// * Math.cos(Math.toRadians(distanceFromClosestIncrement()));
                else dis -= temp[b];// * Math.cos(Math.toRadians(distanceFromClosestIncrement()));
                calcDis[b] = dis;
            }

            /*
            if (useCorrection) {
                for (int i = 0; i < 2; i++) {
                    if (Math.abs(calcDis[i] - currentPosition[i]) > positionSettings.maxPositionChange[i])
                        return;
                }
            }

             */

            distSensorPosition.X = calcDis[0];
            distSensorPosition.Y = calcDis[1];
            distSensorPosition.R = currentPosition.R;
        }
    }

    private void updateDistanceSensor(int sensor)
    {
        sleepTillNextSensorReading();
        robot.robotHardware.distSensors.get(sensor - 1).measureRange();
        lastSensorReadingTime = System.currentTimeMillis();
    }

    private void sleepTillNextSensorReading()
    {
        int timeTillNextRead = positionSettings.minDelayBetweenSensorReadings - (int)(System.currentTimeMillis() - lastSensorReadingTime);
        if(timeTillNextRead > 0) {
            robot.sleep(timeTillNextRead);
        }
    }

    //////////
    //camera//
    //////////
    void initCam()
    {
        if(slamra == null) slamra = new T265Camera(positionSettings.cameraToRobot, positionSettings.encoderMeasurementCovariance, robot.hardwareMap.appContext);
        slamra.start();
    }

    void setCurrentCamPos(Position pos)
    {
        Position curPos = getPositionFromCam();
        cameraOffset.X = -curPos.X + pos.X;
        cameraOffset.Y = -curPos.Y + pos.Y;
        cameraOffset.R = -curPos.R + pos.R;
    }

    private Position getPositionFromCam(){
        T265Camera.CameraUpdate camera = slamra.getLastReceivedCameraUpdate();
        if(camera == null) return new Position();
        return new Position(
            -camera.pose.getTranslation().getY() / Constants.mPerInch,
            camera.pose.getTranslation().getX() / Constants.mPerInch,
            -camera.pose.getRotation().getDegrees()
        );
    }

    void getPosFromCam()
    {
        Position pos = getPositionFromCam();
        pos.add(cameraOffset);
        cameraPosition = pos;
    }

    void endCam() {slamra.stop();}

    /////////
    //files//
    /////////
    public Position getPositionFromFile(){
        String val = FileManager.readFromFile(fileName, AppUtil.getDefContext());
        try {
            String[] vals = val.split(",");
            return new Position(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]), Double.parseDouble(vals[2]));
        }
        catch(Exception e){return null;}
    }
    void writePositionToFile(){
        FileManager.writeToFile(fileName, currentPosition.X + "," + currentPosition.Y + "," + currentPosition.R, AppUtil.getDefContext());
    }

    //////////////////
    //runs in thread//
    //////////////////
    void initializeCurrentPosition()
    {
        if(positionSettings.startPosMode != 1) positionSettings.startPos = new Position();
        if(positionSettings.startPosMode == 2) {
            Position filePos = getPositionFromFile();
            if(filePos != null) positionSettings.startPos = filePos;
        }
        currentPosition = positionSettings.startPos;
    }

    void initializeEncoderTracking()
    {
        currMotorPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
    }

    void updateRot()
    {
        currentAngularVelocity = robot.imu.getAngularVelocity();
        currentAllAxisRotations = updateAngles();
        currentPosition.R = currentAllAxisRotations.thirdAngle;
    }

    void updateAllPos(){

        updateRot();

        if(robot.robotUsage.positionUsage.useDistanceSensors) {
            inMeasuringRange = isRobotInRotationRange();

            if (inMeasuringRange > -2) {
                updateDistanceSensor(1);
            }
        }

        if(robot.robotUsage.positionUsage.useEncoders) getPosFromEncoder();

        if(inMeasuringRange > -2 && robot.robotUsage.positionUsage.useDistanceSensors)
        {
            updateDistanceSensor(2);
        }

        if(robot.robotUsage.positionUsage.usePositionCamera) getPosFromCam();

        if(inMeasuringRange > -2 && robot.robotUsage.positionUsage.useDistanceSensors)
        {
            updatePosWithDistanceSensor(false);
        }
    }

    @Override
    public void run()
    {
        initializeCurrentPosition();
        if(robot.robotUsage.positionUsage.useEncoders) initializeEncoderTracking();
        if(robot.robotUsage.positionUsage.usePositionCamera) setCurrentCamPos(currentPosition);
        isInitialized = true;

        while (!this.isInterrupted() && !robot.opMode.isStopRequested()) {
            updateAllPos();
        }

        if(robot.robotUsage.positionUsage.usePositionCamera) endCam();

        writePositionToFile();
    }

    void waitForPositionInitialization(){
        while(!isInitialized){
            robot.delay(1);
        }
    }

    public void drawPosition(Position pos, Integer color){
        Canvas field = robot.packet.fieldOverlay();
        double robotRadius = 3;

        pos = pos.switchXY();

        Translation2d translation = pos.toPose2d(false).getTranslation();
        Rotation2d rotation = pos.toPose2d(false).getRotation();

        field.setStroke(color.toString());
        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);
    }

    public void drawAllPositions(){
        drawPosition(distSensorPosition, Color.RED);
        drawPosition(encoderPosition, Color.BLUE);
        drawPosition(cameraPosition, Color.GREEN);
    }

    ///////////////////
    //pos with offset//
    ///////////////////
    double[] getPositionWithOffsetArray(double X, double Y, double R) {
        return new double[]{currentPosition.X + X, currentPosition.Y + Y, currentPosition.R + R};
    }
    double[] getPositionWithOffsetArray(Position offset){
        return new double[]{currentPosition.X + offset.X, currentPosition.Y + offset.Y, currentPosition.R + offset.R};
    }
    Position getPositionWithOffset(double X, double Y, double R){
        return new Position(currentPosition.X + X, currentPosition.Y + Y, currentPosition.R + R);
    }
    Position getPositionWithOffset(Position offset){
        return new Position(currentPosition.X + offset.X, currentPosition.Y + offset.Y, currentPosition.R + offset.R);
    }



    ///////////////
    //stop thread//
    ///////////////
    void stopPosition()
    {
        this.interrupt();
    }
}

@Config
class PositionSettings
{
    //////////////////
    //user variables//
    //////////////////
    //positionTracker start
    short startPosMode = 2; //0 = set all to 0, 1 = use variable below, 2 = use file
    Position startPos = new Position(-20, -124, 0);

    //wheel ticks
    public double ticksPerInchForward = 44;
    public double ticksPerInchSideways = 51.3;

    //ultra sonic
    float[][] distancesFromWall = new float[][] //these are the distances that the ultra sonic sensors are at while the robot is at the 0 point and at specific angles
    {
        new float[]{-29.133f, 1.968f}, // for 0 degrees
        new float[]{52.755f,  1.968f}, // for 90 degrees
        new float[]{50.3932f, -127.165f}, // for 180 degrees
        new float[]{-30.314f, -126.771f}  // for -90/270 degrees
    };
    SensorNum[] sensorPosition = new SensorNum[] // which ultra sonic sensor is in the X direction for each 90 degree increment
    {
        SensorNum.ONE, // for 0 degrees
        SensorNum.TWO, // for 90 degrees
        SensorNum.ONE, // for 180 degrees
        SensorNum.TWO  // for -90/270 degrees
    };
    MathSign[][] operations = new MathSign[][] //whether the distance from the ultrasonic sensor should be added or removed for each 90 degree increment
    {
        new MathSign[]{MathSign.ADD, MathSign.SUBTRACT}, // for 0 degrees
        new MathSign[]{MathSign.SUBTRACT, MathSign.SUBTRACT}, // for 90 degrees
        new MathSign[]{MathSign.SUBTRACT, MathSign.ADD}, // for 180 degrees
        new MathSign[]{MathSign.ADD, MathSign.ADD}  // for -90/270 degrees
    };
    double angleTolerance = 15; // how far from each 90 degree increment can the robot be for the ultra sonic to still be valid
    int minDelayBetweenSensorReadings = 50; //how long it should wait to get the distance from last distance reading

    //camera
    double encoderMeasurementCovariance = 0.1;
    Transform2d cameraToRobot = new Transform2d();

    PositionSettings(){}
}

class Position
{
    double X,Y,R;

    Position(double X, double Y, double R){
        this.X = X;
        this.Y = Y;
        this.R = R;
    }
    Position(double[] vals){
        try{
            this.X = vals[0];
            this.Y = vals[1];
            this.R = vals[2];
        }
        catch (Exception e){}
    }
    Position(){
        X = 0;
        Y = 0;
        R = 0;
    }

    double[] toArray()
    {
        try{ return new double[]{X,Y,R}; }
        catch (Exception e){ return null; }
    }

    Position switchXY(){
        return new Position(Y, X, R);
    }

    Position invertX(){
        return new Position(-X, Y, R);
    }

    Position invertY(){
        return new Position(X, -Y, R);
    }

    Position invertR(){
        return new Position(X, Y, -R);
    }

    void add(Position pos2){
        X += pos2.X;
        Y += pos2.Y;
        R += pos2.R;
    }

    void subtract(Position pos2){
        X -= pos2.X;
        Y -= pos2.Y;
        R -= pos2.R;
    }

    Pose2d toPose2d(boolean convertToMeters){
        if(convertToMeters) return new Pose2d(X * Constants.mPerInch, Y * Constants.mPerInch, new Rotation2d(R));
        return new Pose2d(X, Y, new Rotation2d(R));
    }
}

enum MathSign
{
    ADD,
    SUBTRACT
}
enum SensorNum
{
    ONE,
    TWO
}