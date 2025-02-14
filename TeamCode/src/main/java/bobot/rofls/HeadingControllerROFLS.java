package bobot.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import bobot.controllers.HeadingController;

@Config
@TeleOp(group = "!1ROFLS")
public class HeadingControllerROFLS extends LinearOpMode {

    public static double targetHeading = 0;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        HeadingController headingController = new HeadingController(hardwareMap);

        Motor FLMotor = new Motor(hardwareMap, "FLMotor");
        Motor FRMotor = new Motor(hardwareMap, "FRMotor");
        Motor BLMotor = new Motor(hardwareMap, "BLMotor");
        Motor BRMotor = new Motor(hardwareMap, "BRMotor");

        FLMotor.setInverted(true);
        BLMotor.setInverted(true);

        MecanumDrive drive = new MecanumDrive(false, FLMotor, FRMotor, BLMotor, BRMotor);

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        waitForStart();

        double lastTimeStamp = 0;
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

            double period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;

            headingController.setTargetHeading(targetHeading);
            drive.driveRobotCentric(0, 0, headingController.calculate());

            headingController.debug(multipleTelemetry);

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);

            multipleTelemetry.update();
        }
    }
}
