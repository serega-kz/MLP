package bobot.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.ResetOpMode;
import bobot.controllers.YameteKudasai;

@TeleOp(group = "!0teleopus")
public class ResetTeleopus extends ResetOpMode {

    @Override
    public void runOpMode() {
        opMode = YameteKudasai.OpMode.TELEOPUS;
        super.runOpMode();
    }
}
