package org.firstinspires.ftc.teamcode.robot2020;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * manages, reads, and writes files - mainly for persistant values
 */
class FileManager {

    /**
     * reads files as strings
     * 
     * @param fileName the name of the file you want to read
     * @param context  the app context to access the file system
     * @return a string that contains the contents of the file
     */
    static String readFromFile(String fileName, Context context) {

        String ret = null;

        try {
            InputStream inputStream = context.openFileInput(fileName);

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString;
                StringBuilder stringBuilder = new StringBuilder();

                while ((receiveString = bufferedReader.readLine()) != null) {
                    stringBuilder.append("\n").append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }

    /**
     * writes a string of data to an existing file or to a new file
     * 
     * @param fileName the name of the file - if the file does not exist it will
     *                 create a new one
     * @param data     the data(as a string) you want to write to the file
     * @param context  the app context used to access the file system
     */
    static void writeToFile(String fileName, String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(
                    context.openFileOutput(fileName, Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
}

/**
 * constants that are used throughout the code
 */
class Constants {
    public static final float mmPerInch = 25.4f;
    public static final float cmPerInch = 2.54f;
    public static final float mPerInch = 0.0254f;

}

/**
 * class that creates and runs a PID loop(still in development - NOT WORKING)
 */
class PID {
    PIDCoefficients PIDs;
    double maxClamp;
    double minClamp;
    double value;

    double totalError;
    double lastError;
    double currentError;
    long lastTime;

    /**
     * default constructor - makes unusable loop
     */
    PID() {
    }

    /**
     * main constructor that makes a PID loop
     * 
     * @param PIDs     the PID coefficients that get stored and used
     * @param minClamp the minimum value that the loop can have
     * @param maxClamp the maximum value that the loop can have
     */
    PID(PIDCoefficients PIDs, double minClamp, double maxClamp) {
        this.PIDs = PIDs;
        this.minClamp = minClamp;
        this.maxClamp = maxClamp;
    }

    /**
     * updates the PID loop with a new error value
     * 
     * @param error the new error value for the loop
     */
    void updatePID(double error) {
        lastError = currentError;
        currentError = error;
        totalError += error;

        double calculatedI = (totalError * PIDs.i);
        if (calculatedI > maxClamp)
            totalError = maxClamp / PIDs.i;
        else if (calculatedI < minClamp)
            totalError = minClamp / PIDs.i;

        double calculatedD = (((currentError - lastError) * PIDs.d)
                / ((double) (System.nanoTime() - lastTime) / (double) 1000000000));

        value = (error * PIDs.p) + calculatedI - calculatedD;

        lastTime = System.nanoTime();
    }

    /**
     * resets all the errors by setting everything to 0
     */
    void resetErrors() {
        totalError = 0;
        lastError = 0;
        currentError = 0;
    }

    /**
     * updates the PID loop with a new error value and return the output value
     * 
     * @param error the new error value for the loop
     * @return the output value after putting the new error in the loop
     */
    double updatePIDAndReturnValue(double error) {
        updatePID(error);
        return returnValue();
    }

    /**
     * returns the clamped value of the PID loop
     * 
     * @return the clamped value
     */
    double returnValue() {
        return Math.min(Math.max(value, minClamp), maxClamp);
    }

    /**
     * returns the unclamped value of the PID loop
     * 
     * @return the unclamped value
     */
    double returnUncappedValue() {
        return value;
    }
}

/**
 * an enum that stores all the gamepad buttons
 */
enum GamepadButtons {
    dpadUP, dpadDOWN, dpadLEFT, dpadRIGHT,

    A, B, X, Y,

    START, BACK, leftBUMPER, rightBUMPER,

    leftJoyStickX, leftJoyStickY, leftJoyStickBUTTON, leftTRIGGER,

    rightJoyStickX, rightJoyStickY, rightJoyStickBUTTON, rightTRIGGER, combinedTRIGGERS;

    boolean wasButtonPressed = false;
    long lastButtonRelease = System.currentTimeMillis();
}

/**
 * a button/slidder manager that tells whether the button/slider was presses,
 * relesed, held, and the value
 */
class GamepadButtonManager {
    boolean wasButtonPressed = false;
    long lastButtonRelease = System.currentTimeMillis();
    Gamepad gamepad;
    GamepadButtons gamepadButton;
    double minSliderVal = 0.1;

    /**
     * a constructor that stores both the gamepad and the button
     * 
     * @param gamepad       the gamepad you want to use
     * @param gamepadButton the button you want to monitor
     */
    GamepadButtonManager(Gamepad gamepad, GamepadButtons gamepadButton) {
        this.gamepad = gamepad;
        this.gamepadButton = gamepadButton;
    }

    /**
     * a constructor that stores only the button and gets the gamepad passed in from
     * methods
     * 
     * @param gamepadButton the button you want to monitor
     */
    GamepadButtonManager(GamepadButtons gamepadButton) {
        this.gamepadButton = gamepadButton;
    }

    /**
     * checks if the button that is being monitored on the passed in gamepad is
     * pressed
     * 
     * @param gamepad the gamepad you want to use
     * @return whether or not the button was pressed
     */
    boolean getButtonHeld(Gamepad gamepad) {
        if (gamepad.start) {
            if (gamepadButton == GamepadButtons.START)
                return true;
            return false;
        }
        if (gamepadButton == GamepadButtons.A)
            return gamepad.a;
        if (gamepadButton == GamepadButtons.B)
            return gamepad.b;
        if (gamepadButton == GamepadButtons.X)
            return gamepad.x;
        if (gamepadButton == GamepadButtons.Y)
            return gamepad.y;

        if (gamepadButton == GamepadButtons.dpadUP)
            return gamepad.dpad_up;
        if (gamepadButton == GamepadButtons.dpadDOWN)
            return gamepad.dpad_down;
        if (gamepadButton == GamepadButtons.dpadLEFT)
            return gamepad.dpad_left;
        if (gamepadButton == GamepadButtons.dpadRIGHT)
            return gamepad.dpad_right;

        if (gamepadButton == GamepadButtons.leftJoyStickBUTTON)
            return gamepad.left_stick_button;
        if (gamepadButton == GamepadButtons.rightJoyStickBUTTON)
            return gamepad.right_stick_button;
        if (gamepadButton == GamepadButtons.leftBUMPER)
            return gamepad.left_bumper;
        if (gamepadButton == GamepadButtons.rightBUMPER)
            return gamepad.right_bumper;

        if (gamepadButton == GamepadButtons.BACK)
            return gamepad.back;

        if (getSliderValue(gamepad) > minSliderVal)
            return true;

        return false;
    }

    /**
     * checks if the button that is being monitored on the stored gamepad is pressed
     * 
     * @return whether or not the button was pressed
     */
    boolean getButtonHeld() {
        return getButtonHeld(gamepad);
    }

    /**
     * checks if the button that is being monitored on the passed in gamepad has
     * been pressed for a certin amount of time
     * 
     * @param gamepad the gamepad you want to use
     * @param time    the time before button is considered held
     * @return if the monitored button has been held for a certin amount of time
     */
    boolean getButtonHeld(Gamepad gamepad, int time) {
        if (getButtonHeld(gamepad)) {
            return System.currentTimeMillis() - lastButtonRelease > time;
        } else
            lastButtonRelease = System.currentTimeMillis();
        return false;
    }

    /**
     * checks if the button that is being monitored on the stored gamepad has been
     * pressed for a certin amount of time
     * 
     * @param time the time before button is considered held
     * @return if the monitored button has been held for a certin amount of time
     */
    boolean getButtonHeld(int time) {
        return getButtonHeld(gamepad, time);
    }

    /**
     * checks if the button that is being monitored on the passed in gamepad has just been
     * pressed
     * 
     * @param gamepad the gamepad you want to use
     * @return if the monitored button has just been pressed
     */
    boolean getButtonPressed(Gamepad gamepad) {
        if (getButtonHeld(gamepad)) {
            if (!wasButtonPressed) {
                wasButtonPressed = true;
                return true;
            }
        } else
            wasButtonPressed = false;
        return false;
    }

    /**
     * checks if the button that is being monitored on the stored gamepad has just been
     * pressed
     * 
     * @return if the monitored button has just been pressed
     */
    boolean getButtonPressed() {
        return getButtonPressed(gamepad);
    }

    /**
     * checks if the button that is being monitored on the passed in gamepad has just been
     * released
     * 
     * @param gamepad the gamepad you want to use
     * @return if the monitored button has just been released
     */
    boolean getButtonReleased(Gamepad gamepad) {
        if (getButtonHeld(gamepad))
            wasButtonPressed = true;
        else if (wasButtonPressed) {
            wasButtonPressed = false;
            return true;
        }
        return false;
    }

    /**
     * checks if the button that is being monitored on the stored gamepad has just been
     * released
     * 
     * @return if the monitored button has just been released
     */
    boolean getButtonReleased() {
        return getButtonReleased(gamepad);
    }

    /**
     * getts the value of the slider that is being monitored on the passed in gamepad
     * 
     * @param gamepad the gamepad you want to use
     * @return whe value of the monitored slider
     */
    float getSliderValue(Gamepad gamepad) {
        if (gamepadButton == GamepadButtons.leftJoyStickX)
            return gamepad.left_stick_x;
        if (gamepadButton == GamepadButtons.leftJoyStickY)
            return gamepad.left_stick_y;
        if (gamepadButton == GamepadButtons.rightJoyStickX)
            return gamepad.right_stick_x;
        if (gamepadButton == GamepadButtons.rightJoyStickY)
            return gamepad.right_stick_y;

        if (gamepadButton == GamepadButtons.leftTRIGGER)
            return gamepad.left_trigger;
        if (gamepadButton == GamepadButtons.rightTRIGGER)
            return gamepad.right_trigger;
        if (gamepadButton == GamepadButtons.combinedTRIGGERS)
            return gamepad.right_trigger - gamepad.left_trigger;

        return 0;
    }

    /**
     * getts the value of the slider that is being monitored on the stored gamepad
     * 
     * @return whe value of the monitored slider
     */
    float getSliderValue() {
        return getSliderValue(gamepad);
    }

}
