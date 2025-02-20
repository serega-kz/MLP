package bobot.controllers;

import static bobot.controllers.SlideSubController.SlideState.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PDController;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import bobot.utilities.LerpController;

@Config
public class SlideSubController {

    public static double LERP_RATE = 1800, LERP_TOLERANCE = 200;
    public static double DEVIATION_COEFFICIENT = 2500;
    public static double KP = 0.0080, KD = 0.0004;

    private final Motor motor1, motor2;

    private final LerpController lerpController;
    private final PDController PDController1;
    private final PDController PDController2;

    private double scoringPosition = SAMPLE_OUTTAKE_HIGH.targetPosition;

    public SlideSubController(HardwareMap hardwareMap) {
        motor1 = new Motor(hardwareMap, "slideMotor1");
        motor2 = new Motor(hardwareMap, "slideMotor2");
        motor2.setInverted(true);

        lerpController = new LerpController(LERP_RATE, LERP_TOLERANCE);
        lerpController.setStartPosition(motor1.getCurrentPosition());

        PDController1 = new PDController(KP, KD);
        PDController2 = new PDController(KP, KD);
        PDController2.setTolerance(20, 50);
    }

    public boolean isCooking() {
        return lerpController.getProgress() < 1;
    }

    public void start(double startPosition, double endPosition) {
        lerpController.setStartPosition(startPosition);
        lerpController.setEndPosition(endPosition);
    }

    public double getCurrentPosition() {
        return motor1.getCurrentPosition();
    }

    public void deviateTargetPosition(double deviation, double dt) {
        double targetPosition = lerpController.getEndPosition();
        double endPosition = targetPosition + DEVIATION_COEFFICIENT * deviation * dt;
        endPosition = Range.clip(endPosition, 0, SAMPLE_OUTTAKE_HIGH.targetPosition);

        lerpController.setEndPosition(endPosition);
    }

    public void setTargetPosition(double targetPosition) {
        lerpController.setEndPosition(targetPosition);
    }

    public double getScoringPosition() {
        return scoringPosition;
    }

    public void updateScoringPosition() {
        double currentPosition = lerpController.getEndPosition();

        double diff1 = Math.abs(SAMPLE_OUTTAKE_LOW.targetPosition - currentPosition);
        double diff2 = Math.abs(SAMPLE_OUTTAKE_HIGH.targetPosition - currentPosition);

        scoringPosition = diff1 <= diff2 ? SAMPLE_OUTTAKE_LOW.targetPosition : SAMPLE_OUTTAKE_HIGH.targetPosition;
    }

    public void update() {
        PDController1.setPID(KP, 0, KD);
        PDController2.setPID(KP, 0, KD);
        lerpController.setRateAndTolerance(LERP_RATE, LERP_TOLERANCE);

        int motor1Position = motor1.getCurrentPosition();
        int motor2Position = motor2.getCurrentPosition();

        double targetPosition = lerpController.calculate();
        double motor1Power = PDController1.calculate(motor1Position, targetPosition);
        double motor2Power = PDController2.calculate(motor2Position, targetPosition);

        motor1.set(Math.abs(motor1Power) >= 0.05 ? motor1Power : 0);
        motor2.set(Math.abs(motor2Power) >= 0.05 ? motor2Power : 0);
    }

    public void debug(MultipleTelemetry multipleTelemetry) {
        multipleTelemetry.addLine("----- SLIDE CONTROLLER -----");

        multipleTelemetry.addData("SC: lerp progress", lerpController.getProgress());
        multipleTelemetry.addData("SC: target position", lerpController.calculate());

        multipleTelemetry.addData("SC: motor 1 position", motor1.getCurrentPosition());
        multipleTelemetry.addData("SC: motor 2 position", motor2.getCurrentPosition());

        multipleTelemetry.addData("SC: motor 1 current", ((DcMotorEx) motor1.motor).getCurrent(CurrentUnit.MILLIAMPS));
        multipleTelemetry.addData("SC: motor 2 current", ((DcMotorEx) motor2.motor).getCurrent(CurrentUnit.MILLIAMPS));
    }

    public enum SlideState {
        ZERO(0),
        SAMPLE_INTAKE1(400), SAMPLE_INTAKE2(1600), SAMPLE_OUTTAKE(800), SAMPLE_OUTTAKE_LOW(750), SAMPLE_OUTTAKE_HIGH(2100),
        CYCLING1(800), CYCLING2(1600),
        SPECIMEN_INTAKE1(600), SPECIMEN_INTAKE2(1200), SPECIMEN_OUTTAKE(800),
        ASCENT1(1800), ASCENT1_1(200), ASCENT2(600);

        public final int targetPosition;

        SlideState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }
}
