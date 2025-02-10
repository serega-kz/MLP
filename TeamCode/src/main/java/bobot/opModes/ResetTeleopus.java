package bobot.opModes;

import static bobot.controllers.YameteKudasai.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.ResetOpMode;

@TeleOp(group = "!0teleopus")
public class ResetTeleopus extends ResetOpMode {

    @Override
    public void runOpMode() {
        opMode = OpMode.TELEOPUS;
        super.runOpMode();
    }
}
