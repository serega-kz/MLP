package bobot.opModes;

import static bobot.controllers.YameteKudasai.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.Teleopus;

@TeleOp(group = "!0teleopus")
public class TeleopusRed extends Teleopus {

    @Override
    public void runOpMode() {
        alliance = RED;
        super.runOpMode();
    }
}
