package bobot.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import bobot.Teleopus;

@TeleOp(group = "!0teleop")
public class TeleopusBlue extends Teleopus {

    @Override
    public void runOpMode() {
        alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
