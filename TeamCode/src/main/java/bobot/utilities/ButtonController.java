package bobot.utilities;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ButtonController {

    public static double DEBOUNCE = 20, CLICK_TIMEOUT = 250, HOLD_TIMEOUT = 500;

    private final GamepadEx gamepad;
    private final Button button;

    private final ElapsedTime downTimer, upTimer;

    private boolean ignoreRelease = false, ignoreHolding = false;
    private boolean processDC = false, processSC = true;
    private boolean lastButtonValue = false;
    private boolean expectingDC = false;

    public ButtonController(GamepadEx gamepad, Button button) {
        this.gamepad = gamepad;
        this.button = button;

        downTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        upTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public ButtonEvent getButtonEvent() {
        ButtonEvent event = ButtonEvent.NONE;

        boolean currentButtonValue = gamepad.getButton(button);
        if (currentButtonValue && !lastButtonValue && upTimer.time() >= DEBOUNCE) {
            downTimer.reset();
            processDC = upTimer.time() <= CLICK_TIMEOUT && expectingDC && !processDC;

            ignoreHolding = false;
            ignoreRelease = false;
            expectingDC = false;
            processSC = true;
        } else if (!currentButtonValue && lastButtonValue && downTimer.time() >= DEBOUNCE && !ignoreRelease) {
            upTimer.reset();

            if (!processDC) expectingDC = true;
            else {
                expectingDC = false;
                processDC = false;
                processSC = false;

                event = ButtonEvent.DOUBLE_CLICK;
            }
        }

        if (!currentButtonValue && upTimer.time() >= CLICK_TIMEOUT && processSC && expectingDC && !processDC) {
            expectingDC = false;

            event = ButtonEvent.SINGLE_CLICK;
        }

        if (currentButtonValue && downTimer.time() >= HOLD_TIMEOUT && !ignoreHolding) {
            ignoreRelease = true;
            ignoreHolding = true;
            expectingDC = false;
            processDC = false;

            event = ButtonEvent.HOLDING;
        }

        lastButtonValue = currentButtonValue;
        return event;
    }

    public enum ButtonEvent {NONE, SINGLE_CLICK, DOUBLE_CLICK, HOLDING}
}
