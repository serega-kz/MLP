package bobot.controllers;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmSubController {

    private final ColorSensor colorSensor;

    private final Servo shoulderServo1, shoulderServo2;
    private final Servo elbowServo, wristServo, clawServo;

    public ArmSubController(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "armColorSensor");

        shoulderServo1 = hardwareMap.get(Servo.class, "shoulderServo1");
        shoulderServo2 = hardwareMap.get(Servo.class, "shoulderServo2");
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

        if (hsvValues[2] < 100) return SampleColor.NONE;

        if (Math.abs(hsvValues[0] - 17.3) <= 20) return SampleColor.RED; // 17.3 0.611 186
        if (Math.abs(hsvValues[0] - 80.9) <= 20) return SampleColor.YELLOW; // 80.9 0.733 396
        if (Math.abs(hsvValues[0] - 223) <= 20) return SampleColor.BLUE; // 223 0.820 203

        return SampleColor.NONE;
    }

    public void setShoulderPosition(double position) {
        shoulderServo1.setPosition(position);
        shoulderServo2.setPosition(position);
    }

    public void setElbowPosition(double position) {
        elbowServo.setPosition(position);
    }

    public void deviateWristPosition(double deviation) {
        double currentPosition = wristServo.getPosition();
        double targetPosition = currentPosition + deviation;

        if (targetPosition < 0.140) targetPosition = 0.620;
        if (targetPosition > 0.820) targetPosition = 0.340;

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

        float[] hsvValues = getColorSensorHSV();
        multipleTelemetry.addData("AC: sample color", getSampleColor());
        multipleTelemetry.addData("AC: H", hsvValues[0]);
        multipleTelemetry.addData("AC: S", hsvValues[1]);
        multipleTelemetry.addData("AC: V", hsvValues[2]);
    }

    public enum ArmState {
        SAMPLE_INTAKE1(0.480, 0.540, 0.480, 0.315),
        SAMPLE_INTAKE2(0.500, 0.880, 0.480, 0.315),
        SAMPLE_INTAKE3(0.600, 0.870, 0.000, 0.170),
        SAMPLE_OUTTAKE1(0.480, 0.540, 0.480, 0.170),
        SAMPLE_OUTTAKE2(0.480, 0.320, 0.900, 0.315),
        SAMPLE_INTAKE_AUTO(0.000, 0.000, 0.000, 0.000),
        SPECIMEN_INTAKE1(0.080, 0.720, 0.480, 0.315),
        SPECIMEN_INTAKE2(0.080, 0.720, 0.480, 0.170),
        SPECIMEN_OUTTAKE(0.820, 0.160, 0.980, 0.170),
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
