package bobot;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import static bobot.controllers.YameteKudasai.*;
import static bobot.utilities.ButtonController.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import bobot.controllers.YameteKudasai;
import bobot.controllers.HeadingController;
import bobot.utilities.SmartGamepad;

public class Teleopus extends LinearOpMode {

    protected Alliance alliance;

    private double scaleInput(double input, boolean downscale) {
        double coefficient = downscale ? 0.2 : 1;
        return coefficient * Math.abs(input) * input;
    }

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        SmartGamepad driverOp = new SmartGamepad(gamepad1);

        Motor FLMotor = new Motor(hardwareMap, "FLMotor");
        Motor FRMotor = new Motor(hardwareMap, "FRMotor");
        Motor BLMotor = new Motor(hardwareMap, "BLMotor");
        Motor BRMotor = new Motor(hardwareMap, "BRMotor");

        HeadingController headingController = new HeadingController(hardwareMap);
        MecanumDrive mecanumDrive = new MecanumDrive(FLMotor, FRMotor, BLMotor, BRMotor);

        ScoringMode scoringMode = ScoringMode.SAMPLE;
        YameteKudasai やめてください = new YameteKudasai(hardwareMap, alliance, OpMode.TELEOPUS);

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);

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

            boolean downscale = driverOp.getButton(Button.RIGHT_BUMPER) || やめてください.getCurrentState() == State.SAMPLE_INTAKE2;

            double turnSpeed = scaleInput(driverOp.getRightX(), downscale);
            headingController.deviateTargetHeading(turnSpeed, period);

            double strafePower = scaleInput(driverOp.getLeftX(), downscale);
            double forwardPower = scaleInput(driverOp.getLeftY(), downscale);
            double turnPower = headingController.calculate();

            mecanumDrive.driveRobotCentric(strafePower, forwardPower, turnPower);

            if (driverOp.getButtonYEvent() == ButtonEvent.HOLDING) {
                scoringMode = ScoringMode.ASCENT;
                やめてください.setScoringMode(scoringMode);
            } else if (driverOp.getButtonYEvent() == ButtonEvent.SINGLE_CLICK) {
                scoringMode = scoringMode == ScoringMode.SPECIMEN ? ScoringMode.SAMPLE : ScoringMode.SPECIMEN;
                やめてください.setScoringMode(scoringMode);
            } else if (driverOp.getButtonYEvent() == ButtonEvent.DOUBLE_CLICK) {
                scoringMode = ScoringMode.CYCLING;
                やめてください.setScoringMode(scoringMode);
            }

            if (driverOp.wasJustPressed(Button.A)) やめてください.proceedTransition();

            double leftTrigger = scaleInput(driverOp.getTrigger(Trigger.LEFT_TRIGGER), downscale);
            double rightTrigger = scaleInput(driverOp.getTrigger(Trigger.RIGHT_TRIGGER), downscale);

            if (leftTrigger > 0) やめてください.deviateSlideTargetPosition(-leftTrigger, period);
            else if (rightTrigger > 0) やめてください.deviateSlideTargetPosition(rightTrigger, period);

            if (driverOp.wasJustPressed(Button.DPAD_LEFT)) やめてください.rotateWrist(Direction.ANTICLOCKWISE);
            else if (driverOp.wasJustPressed(Button.DPAD_RIGHT)) やめてください.rotateWrist(Direction.CLOCKWISE);

            やめてください.update();
            multipleTelemetry.addData("current state", やめてください.getCurrentState());

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }
}
