package bobot.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PDController;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import bobot.utilities.LerpController;

@Config
public class PivotSubController {

    public static double LERP_RATE = 1200, LERP_TOLERANCE = 200;
    public static double KP = 0.0030, KD = 0.0003;

    private final DigitalChannel touch;

    private final Motor motor1, motor2;

    private final LerpController lerpController;
    private final PDController PDController1;
    private final PDController PDController2;

    public PivotSubController(HardwareMap hardwareMap) {
        touch = hardwareMap.get(DigitalChannel.class, "pivotTouch2");
        touch.setMode(DigitalChannel.Mode.INPUT);

        motor1 = new Motor(hardwareMap, "pivotMotor1");
        motor2 = new Motor(hardwareMap, "pivotMotor2");

        lerpController = new LerpController(LERP_RATE, LERP_TOLERANCE);
        lerpController.setStartPosition(motor1.getCurrentPosition());

        PDController1 = new PDController(KP, KD);
        PDController2 = new PDController(KP, KD);
    }

    public void start() {
        double currentPosition = motor1.getCurrentPosition();
        lerpController.setStartPosition(currentPosition);
        lerpController.setEndPosition(currentPosition);
    }

    public boolean isCooking() {
        return lerpController.getProgress() < 1;
    }

    public void setTargetPosition(double targetPosition) {
        lerpController.setEndPosition(targetPosition);
    }

    public void update() {
        PDController1.setPID(KP, 0, KD);
        PDController2.setPID(KP, 0, KD);
        lerpController.setRateAndTolerance(LERP_RATE, LERP_TOLERANCE);

        if (!touch.getState() && lerpController.getEndPosition() == PivotState.SAMPLE_INTAKE.targetPosition) {
            motor1.resetEncoder();
            motor2.resetEncoder();
            lerpController.reset();
        }

        int motor1Position = motor1.getCurrentPosition();
        int motor2Position = motor2.getCurrentPosition();

        double targetPosition = lerpController.calculate();
        double motor1Power = PDController1.calculate(motor1Position, targetPosition);
        double motor2Power = PDController2.calculate(motor2Position, targetPosition);

        motor1.set(Math.abs(motor1Power) >= 0.05 ? motor1Power : 0);
        motor2.set(Math.abs(motor2Power) >= 0.05 ? motor2Power : 0);
    }

    public void debug(MultipleTelemetry multipleTelemetry) {
        multipleTelemetry.addLine("----- PIVOT CONTROLLER -----");

        multipleTelemetry.addLine("PC: button is " + (touch.getState() ? "not " : "") + "pressed");

        multipleTelemetry.addData("PC: lerp progress", lerpController.getProgress());
        multipleTelemetry.addData("PC: target position", lerpController.calculate());

        multipleTelemetry.addData("PC: motor 1 position", motor1.getCurrentPosition());
        multipleTelemetry.addData("PC: motor 2 position", motor2.getCurrentPosition());

        multipleTelemetry.addData("PC: motor 1 current", ((DcMotorEx) motor1.motor).getCurrent(CurrentUnit.MILLIAMPS));
        multipleTelemetry.addData("PC: motor 2 current", ((DcMotorEx) motor2.motor).getCurrent(CurrentUnit.MILLIAMPS));
    }

    public enum PivotState {
        SAMPLE_INTAKE(-200), SAMPLE_OUTTAKE(1080),
        SPECIMEN_INTAKE(-200), SPECIMEN_OUTTAKE(1180),
        ASCENT1(530), ASCENT1_1(1080), ASCENT2(880), ASCENT3(1080);

        public final int targetPosition;

        PivotState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }
}
