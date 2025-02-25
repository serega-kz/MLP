package bobot;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import static bobot.controllers.YameteKudasai.*;
import static bobot.utilities.ButtonController.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import bobot.controllers.YameteKudasai;
import bobot.controllers.HeadingController;
import bobot.utilities.SmartGamepad;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Teleopus extends LinearOpMode {

    protected Alliance alliance;

    private double getInput(double input, boolean downscale) {
        double coefficient = downscale ? 0.40 : 1;
        return coefficient * Math.abs(input) * input;
    }

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        SmartGamepad driverOp = new SmartGamepad(gamepad1);

        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        follower.startTeleopDrive();

        HeadingController headingController = new HeadingController(hardwareMap);

        ScoringMode scoringMode = ScoringMode.SAMPLE;
        YameteKudasai やめてください = new YameteKudasai(hardwareMap, alliance, OpMode.TELEOPUS);

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);

        hardwareMap.get(LED.class, "LED1").on();
        hardwareMap.get(LED.class, "LED2").on();

        waitForStart();

        double lastTimeStamp = 0;
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

            double period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;

            driverOp.readButtons();

            if (driverOp.wasJustPressed(Button.BACK)) headingController.resetHeading();
            else if (driverOp.wasJustPressed(Button.X)) headingController.snapTargetHeading(HeadingController.Direction.CLOCKWISE);
            else if (driverOp.wasJustPressed(Button.B)) headingController.snapTargetHeading(HeadingController.Direction.ANTICLOCKWISE);

            boolean downscale = driverOp.getButton(Button.RIGHT_BUMPER) ^ やめてください.getCurrentState() == State.SAMPLE_INTAKE2;

            double turnSpeed = getInput(driverOp.getRightX(), downscale);
            headingController.deviateTargetHeading(turnSpeed, period);

            double forwardPower = getInput(driverOp.getLeftY(), downscale);
            double strafePower = getInput(driverOp.getLeftX(), downscale);
            double turnPower = headingController.calculate();

            follower.setTeleOpMovementVectors(forwardPower, strafePower, turnPower, true);
            follower.update();

            ButtonEvent buttonYEvent = driverOp.getButtonYEvent();
            if (buttonYEvent == ButtonEvent.HOLDING) {
                scoringMode = ScoringMode.ASCENT;
                やめてください.setScoringMode(scoringMode);
            } else if (buttonYEvent == ButtonEvent.SINGLE_CLICK) {
                scoringMode = scoringMode == ScoringMode.SPECIMEN ? ScoringMode.SAMPLE : ScoringMode.SPECIMEN;
                やめてください.setScoringMode(scoringMode);
            } else if (buttonYEvent == ButtonEvent.DOUBLE_CLICK) {
                scoringMode = ScoringMode.CYCLING;
                やめてください.setScoringMode(scoringMode);
            }

            if (driverOp.wasJustPressed(Button.A)) やめてください.proceedTransition();

            double leftTrigger = getInput(driverOp.getTrigger(Trigger.LEFT_TRIGGER), downscale);
            double rightTrigger = getInput(driverOp.getTrigger(Trigger.RIGHT_TRIGGER), downscale);

            if (leftTrigger > 0) やめてください.deviateSlideTargetPosition(-leftTrigger, period);
            else if (rightTrigger > 0) やめてください.deviateSlideTargetPosition(rightTrigger, period);

            if (driverOp.wasJustPressed(Button.DPAD_LEFT)) やめてください.rotateWrist(Direction.ANTICLOCKWISE);
            else if (driverOp.wasJustPressed(Button.DPAD_RIGHT)) やめてください.rotateWrist(Direction.CLOCKWISE);

            やめてください.update();
            multipleTelemetry.addData("scoring mode", scoringMode);
            multipleTelemetry.addData("target state", やめてください.getTargetState());
            multipleTelemetry.addData("current state", やめてください.getCurrentState());

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }
}
