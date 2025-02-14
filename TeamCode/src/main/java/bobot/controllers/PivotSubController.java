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

    public static double LERP_RATE = 1600, LERP_TOLERANCE = 200;
    public static double KP = 0.0080, KD = 0.0002;

    private final DigitalChannel touch1, touch2;

    private final Motor motor1, motor2;

    private final LerpController lerpController;
    private final PDController PDController1;
    private final PDController PDController2;

    public PivotSubController(HardwareMap hardwareMap) {
        touch1 = hardwareMap.get(DigitalChannel.class, "pivotTouch1");
        touch2 = hardwareMap.get(DigitalChannel.class, "pivotTouch2");

        touch1.setMode(DigitalChannel.Mode.INPUT);
        touch2.setMode(DigitalChannel.Mode.INPUT);

        motor1 = new Motor(hardwareMap, "pivotMotor1");
        motor2 = new Motor(hardwareMap, "pivotMotor2");

        lerpController = new LerpController(LERP_RATE, LERP_TOLERANCE);
        lerpController.setStartPosition(motor1.getCurrentPosition());

        PDController1 = new PDController(KP, KD);
        PDController2 = new PDController(KP, KD);
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

        if (!touch1.getState() || !touch2.getState()) {
            motor1.resetEncoder();
            motor2.resetEncoder();
            setTargetPosition(0);
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

        multipleTelemetry.addLine("PC: button 1 is " + (touch1.getState() ? "not " : "") + "pressed");
        multipleTelemetry.addLine("PC: button 2 is " + (touch2.getState() ? "not " : "") + "pressed");

        multipleTelemetry.addData("PC: lerp progress", lerpController.getProgress());
        multipleTelemetry.addData("PC: target position", lerpController.calculate());

        multipleTelemetry.addData("PC: motor 1 position", motor1.getCurrentPosition());
        multipleTelemetry.addData("PC: motor 2 position", motor2.getCurrentPosition());

        multipleTelemetry.addData("PC: motor 1 current", ((DcMotorEx) motor1.motor).getCurrent(CurrentUnit.MILLIAMPS));
        multipleTelemetry.addData("PC: motor 2 current", ((DcMotorEx) motor2.motor).getCurrent(CurrentUnit.MILLIAMPS));
    }

    public enum PivotState {
        SAMPLE_INTAKE(-100), SAMPLE_OUTTAKE(1030), SPECIMEN(830), ASCENT1(0), ASCENT2(0);

        public final int targetPosition;

        PivotState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }
}
