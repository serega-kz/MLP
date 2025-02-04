package bobot.utilities;

import static bobot.utilities.ButtonController.*;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class SmartGamepad extends GamepadEx {

    public static double SLEW_RATE = 0.05;

    private final SlewController leftXController, leftYController, rightXController;
    private final ButtonController buttonYController;

    public SmartGamepad(Gamepad gamepad) {
        super(gamepad);

        leftXController = new SlewController(SLEW_RATE);
        leftYController = new SlewController(SLEW_RATE);
        rightXController = new SlewController(SLEW_RATE);

        buttonYController = new ButtonController(this, GamepadKeys.Button.Y);
    }

    @Override
    public void readButtons() {
        leftXController.setSlewRate(SLEW_RATE);
        leftYController.setSlewRate(SLEW_RATE);
        rightXController.setSlewRate(SLEW_RATE);

        super.readButtons();
    }

    @Override
    public double getLeftX() {
        return leftXController.calculate(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return leftYController.calculate(super.getLeftX());
    }

    @Override
    public double getRightX() {
        return rightXController.calculate(super.getLeftX());
    }

    public ButtonEvent getButtonYEvent() {
        return buttonYController.getButtonEvent();
    }
}
