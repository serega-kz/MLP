package bobot.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.Teleopus;

@TeleOp(group = "!0teleop")
public class TeleopusRed extends Teleopus {

    @Override
    public void runOpMode() {
        alliance = Alliance.RED;
        super.runOpMode();
    }
}
