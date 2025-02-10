package bobot.opModes;

import static bobot.controllers.YameteKudasai.Alliance.BLUE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.Teleopus;

@TeleOp(group = "!0teleop")
public class TeleopusBlue extends Teleopus {

    @Override
    public void runOpMode() {
        alliance = BLUE;
        super.runOpMode();
    }
}
