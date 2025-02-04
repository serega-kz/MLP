package bobot.controllers;

import static bobot.controllers.ArmSubController.ArmState.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmSubController {

    public static double WRIST_POSITION_DEVIATION = 0.000;

    private final ColorSensor colorSensor;

    private final Servo shoulderServo1, shoulderServo2;
    private final Servo elbowServo, wristServo, clawServo;

    private double wristScoringPosition = SAMPLE_OUTTAKE2.wristPosition + WRIST_POSITION_DEVIATION;

    public ArmSubController(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "armColor");

        shoulderServo1 = hardwareMap.get(Servo.class, "shoulderServo1");
        shoulderServo2 = hardwareMap.get(Servo.class, "shoulderServo2");
        shoulderServo2.setDirection(Servo.Direction.REVERSE);

        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void setSensorLED(boolean enable) {
        colorSensor.enableLed(enable);
    }

    private float[] getColorSensorHSV() {
        float[] hsvValues = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        return hsvValues;
    }

    public SampleColor getSampleColor() {
        float[] hsvValues = getColorSensorHSV();

        if (hsvValues[2] < 50) return SampleColor.NONE;

        if (hsvValues[0] > 20 && hsvValues[0] < 40) return SampleColor.RED;
        if (hsvValues[0] > 60 && hsvValues[0] < 120) return SampleColor.YELLOW;
        if (hsvValues[0] > 200 && hsvValues[0] < 240) return SampleColor.BLUE;

        return SampleColor.NONE;
    }

    public void setShoulderPosition(double position) {
        shoulderServo1.setPosition(position);
        shoulderServo2.setPosition(position);
    }

    public void setElbowPosition(double position) {
        elbowServo.setPosition(position);
    }

    public double getWristDeviationCount() {
        double wristDeviation = Math.abs(SAMPLE_INTAKE2.wristPosition - wristServo.getPosition());
        return wristDeviation / WRIST_POSITION_DEVIATION;
    }

    public double getWristScoringPosition() {
        wristScoringPosition += WRIST_POSITION_DEVIATION;
        if (wristScoringPosition > SAMPLE_OUTTAKE2.wristPosition) wristScoringPosition -= 3 * WRIST_POSITION_DEVIATION;

        return wristScoringPosition;
    }

    public void deviateWristPosition(double deviation) {
        double currentPosition = wristServo.getPosition();

        double targetPosition = currentPosition + deviation;
        targetPosition = Math.min(targetPosition, SAMPLE_INTAKE2.wristPosition + 2 * WRIST_POSITION_DEVIATION);
        targetPosition = Math.max(targetPosition, SAMPLE_INTAKE2.wristPosition - 2 * WRIST_POSITION_DEVIATION);

        setWristPosition(targetPosition);
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setTargetState(ArmState targetState) {
        setShoulderPosition(targetState.shoulderPosition);
        setElbowPosition(targetState.elbowPosition);
        setWristPosition(targetState.wristPosition);
        setClawPosition(targetState.clawPosition);
    }

    public void debug(MultipleTelemetry multipleTelemetry) {
        multipleTelemetry.addLine("----- ARM CONTROLLER -----");

        multipleTelemetry.addData("AC: sample color", getSampleColor());
    }

    public enum ArmState {
        SAMPLE_INTAKE1(0.000, 0.000, 0.000, 0.000),
        SAMPLE_INTAKE2(0.000, 0.000, 0.000, 0.000),
        SAMPLE_INTAKE3(0.000, 0.000, 0.000, 0.000),
        SAMPLE_OUTTAKE1(0.000, 0.000, 0.000, 0.000),
        SAMPLE_OUTTAKE2(0.000, 0.000, 0.000, 0.000),
        SPECIMEN_INTAKE(0.000, 0.000, 0.000, 0.000),
        SPECIMEN_OUTTAKE(0.000, 0.000, 0.000, 0.000),
        ASCENT(0.000, 0.000, 0.000, 0.000);

        public final double shoulderPosition;
        public final double elbowPosition;
        public final double wristPosition;
        public final double clawPosition;

        ArmState(double shoulderPosition, double elbowPosition, double wristPosition, double clawPosition) {
            this.shoulderPosition = shoulderPosition;
            this.elbowPosition = elbowPosition;
            this.wristPosition = wristPosition;
            this.clawPosition = clawPosition;
        }
    }

    public enum SampleColor {NONE, RED, BLUE, YELLOW}
}
