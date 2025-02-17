package bobot.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import bobot.controllers.ArmSubController;

@Config
@TeleOp(group = "!1ROFLS")
public class ArmSubControllerROFLS extends LinearOpMode {

    public static double shoulderTargetPosition = ArmSubController.ArmState.SAMPLE_INTAKE1.shoulderPosition;
    public static double elbowTargetPosition = ArmSubController.ArmState.SAMPLE_INTAKE1.elbowPosition;
    public static double wristTargetPosition = ArmSubController.ArmState.SAMPLE_INTAKE1.wristPosition;
    public static double clawTargetPosition = ArmSubController.ArmState.SAMPLE_INTAKE1.clawPosition;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        ArmSubController armSubController = new ArmSubController(hardwareMap);

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

            armSubController.setShoulderPosition(shoulderTargetPosition);
            armSubController.setElbowPosition(elbowTargetPosition);
            armSubController.setWristPosition(wristTargetPosition);
            armSubController.setClawPosition(clawTargetPosition);

            armSubController.debug(multipleTelemetry);

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);

            multipleTelemetry.update();
        }
    }
}
