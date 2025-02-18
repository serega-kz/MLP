package bobot.utilities;

import static bobot.utilities.ButtonController.*;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class SmartGamepad extends GamepadEx {

    public static double SLEW_RATE = 0.10;

    private final ButtonController buttonYController;

    private double previousLeftX = 0, previousLeftY = 0, previousRightX = 0;

    public SmartGamepad(Gamepad gamepad) {
        super(gamepad);

        buttonYController = new ButtonController(this, GamepadKeys.Button.Y);
    }

    @Override
    public void readButtons() {
        super.readButtons();
    }

    public double slew(double currentValue, double previousValue) {
        if (SLEW_RATE < Math.abs(currentValue - previousValue)) {
            if (currentValue < previousValue) return previousValue - SLEW_RATE;
            else if (currentValue > previousValue) return previousValue + SLEW_RATE;
        }

        return currentValue;
    }

    @Override
    public double getLeftX() {
        double currentValue = -super.getLeftX();
        double output = slew(currentValue, previousLeftX);

        previousLeftX = output;
        return output;
    }

    @Override
    public double getLeftY() {
        double currentValue = super.getLeftY();
        double output = slew(currentValue, previousLeftY);

        previousLeftY = output;
        return output;
    }

    @Override
    public double getRightX() {
        double currentValue = super.getRightX();
        double output = slew(currentValue, previousRightX);

        previousRightX = output;
        return output;
    }

    public ButtonEvent getButtonYEvent() {
        return buttonYController.getButtonEvent();
    }
}
