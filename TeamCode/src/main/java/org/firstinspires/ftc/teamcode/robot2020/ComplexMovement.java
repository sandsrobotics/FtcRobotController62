package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot2020.persistence.Movement.MovementEntity;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;

@Config
public class ComplexMovement {

    //////////////////
    // user variables//
    //////////////////
    public static double measureDelay = 0; // in ms
    public static double maxTime = 6000; // in ms

    ///////////////////
    // other variables//
    ///////////////////
    // make
    protected List<int[]> positions = new ArrayList<>();
    protected boolean isRecording = false;
    protected double curRecordingLength = 0;
    protected boolean startOfRecording = true;
    protected int[] motorStartOffset;
    long startMs;
    // load
    protected List<int[]> loaded_Positions = new ArrayList<>();
    protected double loaded_MeasureDelay;
    protected double loaded_TotalTime;

    // other class objects
    Robot robot;

    ComplexMovement(Robot robot) {
        this.robot = robot;
    }

    void recorder(boolean stopAtMaxTime) {
        if (measureDelay > maxTime && stopAtMaxTime && robot.robotSettings.debug_methods)
            robot.addTelemetry("error in ComplexMovement.recorder: ",
                    "measure delay is more than max length so can not record");
        if (isRecording) {
            if (curRecordingLength + measureDelay > maxTime && stopAtMaxTime) {
                if (robot.robotSettings.debug_methods)
                    robot.addTelemetry("ComplexMovement.recorder has stopped recording: ",
                            "this recording has stopped at time " + curRecordingLength
                                    + " ms: stop recording to make file");
                isRecording = false;
            } else {
                if (startOfRecording) {
                    motorStartOffset = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
                    startMs = System.currentTimeMillis();
                    startOfRecording = false;
                }

                int[] pos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
                for (int i = 0; i < pos.length; i++)
                    pos[i] -= motorStartOffset[i];
                positions.add(pos);
            }
            if (measureDelay > 0)
                sleep((long) measureDelay);
            curRecordingLength = System.currentTimeMillis() - startMs;
        }
    }

    void startRecording() {
        resetRecording();
        robot.robotHardware.setMotorsZeroPowerBehaviorList(robot.robotHardware.driveMotors,
                DcMotor.ZeroPowerBehavior.FLOAT);
        isRecording = true;
    }

    void stopRecording(boolean makeFile, String moveName) {
        if (makeFile) {
            makeFile(moveName);
        }
        isRecording = false;
    }

    void resetRecording() {
        positions.clear();
        curRecordingLength = 0;
        startOfRecording = true;
        motorStartOffset = null;
    }

    void makeFile(String moveName) {
        if (moveName == null || moveName.equals(""))
            moveName = "not named";
        MovementEntity entity = new MovementEntity(moveName, 0, (int) curRecordingLength);
        robot.db.movementEntityDAO().insertAll(entity);
        entity = new MovementEntity(moveName, 0, (float) (curRecordingLength / positions.size()));
        robot.db.movementEntityDAO().insertAll(entity);
        for (int i = 0; i < positions.size(); i++) {
            for (int m = 0; m < robot.robotHardware.driveMotors.size(); m++) {
                MovementEntity entity1 = new MovementEntity(moveName, m + 1, positions.get(i)[m]);
                robot.db.movementEntityDAO().insertAll(entity1);
            }
        }
    }

    void loadMoveDB(String moveName) {
        try {
            List<MovementEntity> data = robot.db.movementEntityDAO().loadMovementByName(moveName);
            int[] currentLoadTick = new int[4];
            loaded_Positions.clear();
            int i = 0;

            for (MovementEntity m : data) {
                if (m.motor_id == 0) {
                    if (i == 0) {
                        loaded_TotalTime = m.motor_tick;
                        i++;
                    } else if (i == 1) {
                        loaded_MeasureDelay = m.motor_tick;
                        i++;
                    } else
                        break;
                } else {
                    currentLoadTick[m.motor_id - 1] = (int) (m.motor_tick);
                    if (m.motor_id == 4) {
                        loaded_Positions.add(currentLoadTick);
                        currentLoadTick = new int[4];
                    }
                }
            }
        } catch (Exception e) {
            robot.addTelemetry("error in loadMoveDB", "there was no move with the specified tile in the database");
        }
    }

    /*
     * void loadMoveCSV(String fileName) { try { BufferedReader reader = new
     * BufferedReader(new InputStreamReader(getClass().getResourceAsStream("assets/"
     * + fileName))); String Line; int[] currentLoadTick = new int[4]; double[]
     * currentLoadVelocity = new double[4]; int i = 0; loaded_TotalMeasureDelay = 0;
     * 
     * while ((Line = reader.readLine()) != null) { String[] elements =
     * Line.split(","); if(Integer.parseInt(elements[1]) == 0) { if(i == 0)
     * loaded_TotalMeasureDelay += Double.parseDouble(elements[3]); else {
     * loaded_TotalTime = Double.parseDouble(elements[2]); loaded_TotalMeasureDelay
     * += Double.parseDouble(elements[3]); } i++; } else {
     * currentLoadTick[currentLoadTick.length] = Integer.parseInt(elements[2]);
     * currentLoadVelocity[currentLoadVelocity.length] =
     * Double.parseDouble(elements[3]); if(Integer.parseInt(elements[1]) == 4) {
     * loaded_Positions.add(currentLoadTick);
     * loaded_Velocities.add(currentLoadVelocity); currentLoadTick = new int[4];
     * currentLoadVelocity = new double[4]; } } } } catch (IOException e) {
     * if(robot.debug_methods)robot.addTelemetryString("error", e.toString()); } }
     */

    /*
     * void scaleLoadedMove(boolean scaleToMaxPower) { double maxMeasuredVelocity =
     * 0; for(double[] line:loaded_Velocities) { for(double value:line) if(value >
     * maxMeasuredVelocity) maxMeasuredVelocity = value; } if(maxMeasuredVelocity >
     * maxVelocity || scaleToMaxPower) { double multiplier =
     * maxVelocity/maxMeasuredVelocity; loaded_TotalTime /= multiplier;
     * loaded_MeasureDelay /= multiplier;
     * 
     * for(int l = 0; l < loaded_Velocities.size(); l++) { for(int v = 0; v <
     * loaded_Velocities.get(0).length; v++) { loaded_Velocities.get(l)[v] *=
     * multiplier; loaded_Positions.get(l)[v] *= multiplier; } } } }
     * 
     * 
     */

    void clearLoadedMove() {
        loaded_Positions.clear();
        loaded_MeasureDelay = 0;
        loaded_TotalTime = 0;
    }

    void runLoadedMoveV2(boolean stopIfTimeIsMoreThanMoveTime) {
        if (loaded_Positions.size() != 0) {
            int curInstruction = 0;
            int[] motorStartPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
            boolean start = true;
            robot.robotHardware.setMotorsZeroPowerBehaviorList(robot.robotHardware.driveMotors,
                    DcMotor.ZeroPowerBehavior.BRAKE);

            double startMs = System.currentTimeMillis();

            while (!robot.stop()) {
                if (curInstruction * loaded_MeasureDelay <= System.currentTimeMillis() - startMs) {
                    for (int m = 0; m < 4; m++) {
                        robot.robotHardware.driveMotors.get(m)
                                .setTargetPosition(loaded_Positions.get(curInstruction)[m] + motorStartPos[m]);
                    }
                    curInstruction++;
                }
                if (start) {
                    robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 1);
                    robot.robotHardware.setMotorsRunModeList(robot.robotHardware.driveMotors,
                            DcMotor.RunMode.RUN_TO_POSITION);
                    start = false;
                }
                if (curInstruction == loaded_Positions.size() || robot.stop())
                    break;
                if (stopIfTimeIsMoreThanMoveTime && System.currentTimeMillis() - startMs >= loaded_TotalTime) {
                    if (robot.robotSettings.debug_methods)
                        robot.addTelemetry("error in ComplexMovement.runLoadedMoveV2: ",
                                "this move took longer than expected to run. Ending move...");
                    break;
                }
            }
            while (!robot.robotHardware.motorPositionsInToleranceList(robot.robotHardware.driveMotors, 5)
                    && !stopIfTimeIsMoreThanMoveTime) {
                if (robot.stop())
                    break;
            }
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
            robot.robotHardware.setMotorsRunModeList(robot.robotHardware.driveMotors,
                    DcMotor.RunMode.RUN_USING_ENCODER);
            robot.robotHardware.setMotorsToPowerList(robot.robotHardware.driveMotors, 0);
        } else if (robot.robotSettings.debug_methods)
            robot.addTelemetry("warning in ComplexMovement.RunMoveV2: ", "no loaded move!");
    }

    void clearDatabase() {
        robot.db.movementEntityDAO().deleteAll();
    }
}
