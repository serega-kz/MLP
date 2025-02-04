package bobot.opModes;

import static java.lang.Math.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "!0autonomous")
public class AutonomousSpecimen extends LinearOpMode {

    private final Timer pathTimer = new Timer();
    private final Timer actionTimer = new Timer();

    private final Pose startPose = new Pose(0, 0, toRadians(0));

    private PathState pathState;

    private void setPathState(PathState pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();
        setPathState(PathState.SCORE_PRELOAD);

        while (opModeIsActive()) {
            if (pathState == PathState.SCORE_PRELOAD) {

            } else if (pathState == PathState.NIGGA_MOO) {

            } else if (pathState == PathState.SCORE_SPECIMEN) {

            } else if (pathState == PathState.PICK_SPECIMEN) {

            }

            follower.update();
        }
    }

    private enum PathState {SCORE_PRELOAD, NIGGA_MOO, SCORE_SPECIMEN, PICK_SPECIMEN}
}
