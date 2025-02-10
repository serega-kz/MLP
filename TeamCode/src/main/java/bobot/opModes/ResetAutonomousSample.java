package bobot.opModes;

import static bobot.controllers.YameteKudasai.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.ResetOpMode;

@TeleOp(group = "!0teleopus")
public class ResetAutonomousSample extends ResetOpMode {

    @Override
    public void runOpMode() {
        opMode = OpMode.AUTONOMOUS_SAMPLE;
        super.runOpMode();
    }
}
