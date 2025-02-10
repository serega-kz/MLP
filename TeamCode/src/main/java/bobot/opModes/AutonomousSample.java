package bobot.opModes;

import static java.lang.Math.*;
import static bobot.controllers.YameteKudasai.Alliance.*;
import static bobot.controllers.YameteKudasai.State.*;
import static bobot.opModes.AutonomousSample.PathState.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import bobot.controllers.YameteKudasai;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "!0autonomous")
public class AutonomousSample extends OpMode {

    private List<LynxModule> allHubs;
    private double lastTimeStamp;

    private YameteKudasai やめてください;
    private Follower follower;
    private Timer pathTimer;

    private VoltageSensor voltageSensor;
    private MultipleTelemetry multipleTelemetry;

    private final Pose startPose = new Pose(7.900, 113.500, 0);
    private final Pose scorePose = new Pose(14.000, 130.000, -PI / 4);

    private final Pose score1CP = new Pose(24.000, 120.000);

    private final Pose pickup1Pose = new Pose(30.000, 121.500, 0);
    private final Pose pickup2Pose = new Pose(30.000, 131.500, 0);

    private final Pose pickup3CP = new Pose(48.000, 108.000);
    private final Pose pickup3Pose = new Pose(45.500, 135.000, -PI / 2);
    private final Pose score4CP = new Pose(48.000, 108.000);

    private final Pose parkCP = new Pose(72.000, 120.000);
    private final Pose parkPose = new Pose(60.000, 100.000, -PI / 2);

    private Path scorePreload, park;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private PathState pathState;

    private void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(score1CP), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup3CP), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup3Pose), new Point(score4CP), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkCP), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private void autonomousPathUpdate() {
        if (pathState == SCORE_PRELOAD) {
            follower.followPath(scorePreload, true);
            setPathState(SCORE_PRELOAD_1);
        } else if (pathState == SCORE_PRELOAD_1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(GRAB_PICKUP1);
        } else if (pathState == GRAB_PICKUP1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.followPath(grabPickup1, true);
            setPathState(GRAB_PICKUP1_1);
        } else if (pathState == GRAB_PICKUP1_1) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE1) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(GRAB_PICKUP1_2);
        } else if (pathState == GRAB_PICKUP1_2) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE2) return;

            やめてください.proceedTransition();
            setPathState(SCORE_PICKUP1);
        } else if (pathState == SCORE_PICKUP1) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE3) return;

            follower.followPath(scorePickup1);
            setPathState(SCORE_PICKUP1_1);
        } else if (pathState == SCORE_PICKUP1_1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(GRAB_PICKUP2);
        } else if (pathState == GRAB_PICKUP2) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.followPath(grabPickup2, true);
            setPathState(GRAB_PICKUP2_1);
        } else if (pathState == GRAB_PICKUP2_1) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE1) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(GRAB_PICKUP2_2);
        } else if (pathState == GRAB_PICKUP2_2) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE2) return;

            やめてください.proceedTransition();
            setPathState(SCORE_PICKUP2);
        } else if (pathState == SCORE_PICKUP2) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE3) return;

            follower.followPath(scorePickup2);
            setPathState(SCORE_PICKUP2_1);
        } else if (pathState == SCORE_PICKUP2_1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedAutoTransition();
            setPathState(GRAB_PICKUP3);
        } else if (pathState == GRAB_PICKUP3) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.followPath(grabPickup3, true);
            setPathState(GRAB_PICKUP3_1);
        } else if (pathState == GRAB_PICKUP3_1) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE_AUTO) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(SCORE_PICKUP3);
        } else if (pathState == SCORE_PICKUP3) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE_AUTO_2) return;

            follower.followPath(scorePickup3);
            setPathState(SCORE_PICKUP3_1);
        } else if (pathState == SCORE_PICKUP3_1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(PARK);
        } else if (pathState == PARK) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.followPath(park);
            setPathState(AUTONOMOUS_FINISHED);
        }
    }

    private void setPathState(PathState pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        やめてください = new YameteKudasai(hardwareMap, NONE, YameteKudasai.OpMode.AUTONOMOUS_SPECIMEN);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new com.pedropathing.util.Timer();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(SCORE_PRELOAD);
        lastTimeStamp = 0;
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        follower.update();
        やめてください.update();
        autonomousPathUpdate();

        multipleTelemetry.addData("current state", やめてください.getCurrentState());
        multipleTelemetry.addData("path state", pathState);

        double frequency = 1 / period;
        double voltage = voltageSensor.getVoltage();

        multipleTelemetry.addData("frequency", frequency);
        multipleTelemetry.addData("voltage", voltage);

        multipleTelemetry.update();
    }

    public enum PathState {
        SCORE_PRELOAD,
        SCORE_PRELOAD_1,
        GRAB_PICKUP1,
        GRAB_PICKUP1_1,
        GRAB_PICKUP1_2,
        SCORE_PICKUP1,
        SCORE_PICKUP1_1,
        GRAB_PICKUP2,
        GRAB_PICKUP2_1,
        GRAB_PICKUP2_2,
        SCORE_PICKUP2,
        SCORE_PICKUP2_1,
        GRAB_PICKUP3,
        GRAB_PICKUP3_1,
        SCORE_PICKUP3,
        SCORE_PICKUP3_1,
        PARK,
        AUTONOMOUS_FINISHED
    }
}
