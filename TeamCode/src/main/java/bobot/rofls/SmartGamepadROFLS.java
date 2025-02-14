package bobot.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import bobot.utilities.ButtonController.ButtonEvent;
import bobot.utilities.SmartGamepad;

import java.util.List;

@TeleOp(group = "!1ROFLS")
public class SmartGamepadROFLS extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        SmartGamepad driverOp = new SmartGamepad(gamepad1);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        telemetry.setAutoClear(false);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            driverOp.readButtons();
            ButtonEvent buttonYEvent = driverOp.getButtonYEvent();

            if (buttonYEvent == ButtonEvent.HOLDING) multipleTelemetry.addLine("Holding");
            else if (buttonYEvent == ButtonEvent.SINGLE_CLICK) multipleTelemetry.addLine("Single click");
            else if (buttonYEvent == ButtonEvent.DOUBLE_CLICK) multipleTelemetry.addLine("Double click");

            dashboardTelemetry.addData("Left X", driverOp.getLeftX());
            dashboardTelemetry.addData("Left Y", driverOp.getLeftY());
            dashboardTelemetry.addData("Right X", driverOp.getRightX());

            multipleTelemetry.update();
        }
    }
}
