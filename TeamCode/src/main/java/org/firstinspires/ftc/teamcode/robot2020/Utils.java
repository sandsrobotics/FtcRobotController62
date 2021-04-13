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


class FileManager {
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
    static void writeToFile(String fileName, String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(fileName, Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
}


class Constants
{
    public static final float mmPerInch = 25.4f;
    public static final float cmPerInch = 2.54f;
    public static final float mPerInch =  0.0254f;

}

class PID
{
    PIDCoefficients PIDs;
    double maxClamp;
    double minClamp;
    double value;

    double totalError;
    double lastError;
    double currentError;
    long lastTime;

    PID(){}
    PID(PIDCoefficients PIDs, double minClamp, double maxClamp)
    {
        this.PIDs = PIDs;
        this.minClamp = minClamp;
        this.maxClamp = maxClamp;
    }

    void updatePID(double error)
    {
        lastError = currentError;
        currentError = error;
        totalError += error;

        double calculatedI = (totalError * PIDs.i);
        if(calculatedI > maxClamp) totalError = maxClamp/PIDs.i;
        else if(calculatedI < minClamp) totalError = minClamp/ PIDs.i;

        double calculatedD = (((currentError - lastError) * PIDs.d) / ((double)(System.nanoTime() - lastTime) / (double) 1000000000));

        value = (error * PIDs.p) + calculatedI - calculatedD;

        lastTime = System.nanoTime();
    }

    void resetErrors()
    {
        totalError = 0;
        lastError = 0;
        currentError = 0;
    }

    double updatePIDAndReturnValue(double error)
    {
        updatePID(error);
        return returnValue();
    }

    double returnValue()
    {
        return Math.min(Math.max(value, minClamp), maxClamp);
    }

    double returnUncappedValue()
    {
        return value;
    }
}

enum GamepadButtons
{
    dpadUP,
    dpadDOWN,
    dpadLEFT,
    dpadRIGHT,

    A,
    B,
    X,
    Y,

    START,
    BACK,
    leftBUMPER,
    rightBUMPER,

    leftJoyStickX,
    leftJoyStickY,
    leftJoyStickBUTTON,
    leftTRIGGER,

    rightJoyStickX,
    rightJoyStickY,
    rightJoyStickBUTTON,
    rightTRIGGER,
    combinedTRIGGERS;

    boolean wasButtonPressed = false;
    long lastButtonRelease = System.currentTimeMillis();
}

class GamepadButtonManager
{
    boolean wasButtonPressed = false;
    long lastButtonRelease = System.currentTimeMillis();
    Gamepad gamepad;
    GamepadButtons gamepadButton;
    double minSliderVal = 0.1;

    GamepadButtonManager(Gamepad gamepad, GamepadButtons gamepadButton)
    {
        this.gamepad = gamepad;
        this.gamepadButton = gamepadButton;
    }
    GamepadButtonManager(GamepadButtons gamepadButton) { this.gamepadButton = gamepadButton; }

    boolean getButtonHeld(Gamepad gamepad)
    {
        if(gamepad.start){
            if(gamepadButton == GamepadButtons.START) return true;
            return false;
        }
        if(gamepadButton == GamepadButtons.A) return gamepad.a;
        if(gamepadButton == GamepadButtons.B) return gamepad.b;
        if(gamepadButton == GamepadButtons.X) return gamepad.x;
        if(gamepadButton == GamepadButtons.Y) return gamepad.y;

        if(gamepadButton == GamepadButtons.dpadUP) return gamepad.dpad_up;
        if(gamepadButton == GamepadButtons.dpadDOWN) return gamepad.dpad_down;
        if(gamepadButton == GamepadButtons.dpadLEFT) return gamepad.dpad_left;
        if(gamepadButton == GamepadButtons.dpadRIGHT) return gamepad.dpad_right;

        if(gamepadButton == GamepadButtons.leftJoyStickBUTTON) return gamepad.left_stick_button;
        if(gamepadButton == GamepadButtons.rightJoyStickBUTTON) return gamepad.right_stick_button;
        if(gamepadButton == GamepadButtons.leftBUMPER) return gamepad.left_bumper;
        if(gamepadButton == GamepadButtons.rightBUMPER) return gamepad.right_bumper;


        if(gamepadButton == GamepadButtons.BACK) return gamepad.back;

        if(getSliderValue(gamepad) > minSliderVal) return true;

        return false;
    }
    boolean getButtonHeld(){return getButtonHeld(gamepad);}

    boolean getButtonHeld(Gamepad gamepad, int time)
    {
        if(getButtonHeld(gamepad))
        {
            return System.currentTimeMillis() - lastButtonRelease > time;
        }
        else lastButtonRelease = System.currentTimeMillis();
        return false;
    }
    boolean getButtonHeld(int time){return getButtonHeld(gamepad, time);}

    boolean getButtonPressed(Gamepad gamepad)
    {
        if(getButtonHeld(gamepad))
        {
            if(!wasButtonPressed)
            {
                wasButtonPressed = true;
                return true;
            }
        }
        else wasButtonPressed = false;
        return false;
    }
    boolean getButtonPressed(){return getButtonPressed(gamepad);}

    boolean getButtonReleased(Gamepad gamepad)
    {
        if(getButtonHeld(gamepad)) wasButtonPressed = true;
        else if(wasButtonPressed)
        {
            wasButtonPressed = false;
            return true;
        }
        return false;
    }
    boolean getButtonReleased(){return getButtonReleased(gamepad);}

    float getSliderValue(Gamepad gamepad)
    {
        if(gamepadButton == GamepadButtons.leftJoyStickX) return gamepad.left_stick_x;
        if(gamepadButton == GamepadButtons.leftJoyStickY) return gamepad.left_stick_y;
        if(gamepadButton == GamepadButtons.rightJoyStickX) return gamepad.right_stick_x;
        if(gamepadButton == GamepadButtons.rightJoyStickY) return gamepad.right_stick_y;

        if(gamepadButton == GamepadButtons.leftTRIGGER) return gamepad.left_trigger;
        if(gamepadButton == GamepadButtons.rightTRIGGER) return gamepad.right_trigger;
        if(gamepadButton == GamepadButtons.combinedTRIGGERS) return gamepad.right_trigger - gamepad.left_trigger;

        return 0;
    }
    float getSliderValue(){return getSliderValue(gamepad);}

}
