package bobot.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PDController;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import bobot.utilities.LerpController;

@Config
public class HeadingController {

    public static double LERP_RATE = 1, LERP_TOLERANCE = 0;
    public static double DEVIATION_COEFFICIENT = 250;
    public static double KP = 0.040, KD = 0.002;

    private final GoBildaPinpointDriver odo;

    private final PDController headingController;
    private final LerpController lerpController;

    private double integratedHeading = 0, previousHeading = 0;

    public HeadingController(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        headingController = new PDController(KP, KD);
        lerpController = new LerpController(LERP_RATE, LERP_TOLERANCE);
    }

    public void resetHeading() {
        integratedHeading = 0;

        headingController.reset();
        lerpController.reset();
    }

    public void deviateTargetHeading(double deviation, double dt) {
        double targetHeading = lerpController.getEndPosition() - DEVIATION_COEFFICIENT * deviation * dt;
        lerpController.setEndPosition(targetHeading);
    }

    public void snapTargetHeading(Direction direction) {
        double targetHeading = lerpController.getEndPosition();
        if (targetHeading % 90 == 0) {
            int deviation = direction == Direction.ANTICLOCKWISE ? -90 : 90;
            lerpController.setEndPosition(targetHeading + deviation);
        } else if (direction == Direction.ANTICLOCKWISE) {
            lerpController.setEndPosition(Math.floor(targetHeading / 90) * 90);
        } else if (direction == Direction.CLOCKWISE) {
            lerpController.setEndPosition(Math.ceil(targetHeading / 90) * 90);
        }
    }

    public void setTargetHeading(double targetHeading) {
        lerpController.setEndPosition(targetHeading);
    }

    public double calculate() {
        headingController.setPID(KP, 0, KD);
        lerpController.setRateAndTolerance(LERP_RATE, LERP_TOLERANCE);

        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

        double currentHeading = Math.toDegrees(odo.getHeading());
        double deltaHeading = AngleUnit.normalizeDegrees(currentHeading - previousHeading);

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return -headingController.calculate(integratedHeading, lerpController.getEndPosition());
    }

    public void debug(MultipleTelemetry multipleTelemetry) {
        multipleTelemetry.addLine("----- HEADING headingController -----");

        multipleTelemetry.addData("HC: lerp progress", lerpController.getProgress());
        multipleTelemetry.addData("HC: target heading", lerpController.getEndPosition());

        multipleTelemetry.addData("HC: integrated heading", integratedHeading);
    }

    public enum Direction {ANTICLOCKWISE, CLOCKWISE}
}
