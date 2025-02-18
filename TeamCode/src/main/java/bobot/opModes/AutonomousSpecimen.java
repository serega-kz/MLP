package bobot.opModes;

import static java.lang.Math.PI;
import static bobot.controllers.YameteKudasai.*;
import static bobot.opModes.AutonomousSpecimen.PathState.*;

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
public class AutonomousSpecimen extends OpMode {

    private List<LynxModule> allHubs;
    private double lastTimeStamp;

    private YameteKudasai やめてください;
    private Follower follower;
    private Timer pathTimer;

    private VoltageSensor voltageSensor;
    private MultipleTelemetry multipleTelemetry;

    private final Pose startPose = new Pose(7.9, 65.6, PI);
    private final Pose scorePose = new Pose(40.0, 65.6, PI);

    private final Pose pickup0CP = new Pose(24.0, 72.0, PI);
    private final Pose pickup0Pose = new Pose(36.0, 36.0, PI);

    private final Pose pickup1CP = new Pose(62.0, 38.0, PI);
    private final Pose pickup1Pose1 = new Pose(62.0, 22.5, PI);
    private final Pose pickup1Pose2 = new Pose(18.0, 22.5, PI);

    private final Pose pickup2CP = new Pose(62.0, 28.0, PI);
    private final Pose pickup2Pose1 = new Pose(62.0, 12.5, PI);
    private final Pose pickup2Pose2 = new Pose(18.0, 12.5, PI);

    private final Pose pickup3CP = new Pose(62.0, 17.5, PI);
    private final Pose pickup3Pose1 = new Pose(62.0, 7.5, PI);
    private final Pose pickup3Pose2 = new Pose(18.0, 7.5, PI);

    private final Pose specimenCP = new Pose(18.0, 72.0, PI);
    private final Pose specimenGrabPose = new Pose(24.0, 30.0, PI);
    private final Pose specimenScorePose = new Pose(40.0, 70.0, PI);

    private final Pose parkPose = new Pose(12.0, 30.0, 0);

    private Path scorePreload, scoreSpecimen, grabSpecimen, park;
    private PathChain grabPickup;

    private PathState pathState;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup0CP), new Point(pickup0Pose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierCurve(new Point(pickup0Pose), new Point(pickup1CP), new Point(pickup1Pose1)))
                .setConstantHeadingInterpolation(pickup0Pose.getHeading())
                .addPath(new BezierLine(new Point(pickup1Pose1), new Point(pickup1Pose2)))
                .setConstantHeadingInterpolation(pickup1Pose1.getHeading())
                .addPath(new BezierCurve(new Point(pickup1Pose2), new Point(pickup2CP), new Point(pickup2Pose1)))
                .setConstantHeadingInterpolation(pickup1Pose2.getHeading())
                .addPath(new BezierLine(new Point(pickup2Pose1), new Point(pickup2Pose2)))
                .setConstantHeadingInterpolation(pickup2Pose1.getHeading())
                .addPath(new BezierCurve(new Point(pickup2Pose2), new Point(pickup3CP), new Point(pickup3Pose1)))
                .setConstantHeadingInterpolation(pickup2Pose2.getHeading())
                .addPath(new BezierLine(new Point(pickup3Pose1), new Point(pickup3Pose2)))
                .setConstantHeadingInterpolation(pickup3Pose1.getHeading())
                .addPath(new BezierLine(new Point(pickup3Pose2), new Point(specimenGrabPose)))
                .setConstantHeadingInterpolation(pickup3Pose2.getHeading())

                .build();

        scoreSpecimen = new Path(new BezierCurve(new Point(specimenGrabPose), new Point(specimenCP), new Point(specimenScorePose)));
        scoreSpecimen.setConstantHeadingInterpolation(specimenGrabPose.getHeading());

        grabSpecimen = new Path(new BezierCurve(new Point(scorePose), new Point(specimenCP), new Point(specimenGrabPose)));
        grabSpecimen.setConstantHeadingInterpolation(parkPose.getHeading());

        park = new Path(new BezierLine(new Point(specimenGrabPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(specimenGrabPose.getHeading(), parkPose.getHeading());
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private void autonomousPathUpdate() {
        if (pathState == SCORE_PRELOAD) {
            if (pathTimer.getElapsedTime() <= 750) return;

            follower.followPath(scorePreload);
            setPathState(GRAB_PICKUP);
        } else if (pathState == GRAB_PICKUP) {
            if (isFollowerCooking()) return;

//            やめてください.proceedTransition(false);
            follower.followPath(grabPickup, true);
            setPathState(PICK_SPECIMEN1);
        } else if (pathState == PICK_SPECIMEN1 || pathState == PICK_SPECIMEN2 || pathState == PICK_SPECIMEN3 || pathState == PICK_SPECIMEN4) {
            if (isFollowerCooking()) return;
            
            follower.followPath(scoreSpecimen);
            setPathState(PathState.values()[pathState.ordinal() + 1]);
      } else if (pathState == SCORE_SPECIMEN1 || pathState == SCORE_SPECIMEN2 || pathState == SCORE_SPECIMEN3 || pathState == SCORE_SPECIMEN4) {
            if (isFollowerCooking()) return;

            follower.followPath(grabSpecimen, true);
            setPathState(PathState.values()[pathState.ordinal() + 1]);
        } else if (pathState == PARK) {
            if (isFollowerCooking()) return;

            follower.followPath(park, true);
            setPathState(TRANSITION_TO_TELEOPUS);
        }
        else if (pathState == TRANSITION_TO_TELEOPUS) {
            if (isFollowerCooking()) return;

//            やめてください.setScoringMode(ScoringMode.SAMPLE);
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

        pathTimer = new Timer();

//        やめてください = new YameteKudasai(hardwareMap, NONE, YameteKudasai.OpMode.AUTONOMOUS_SPECIMEN);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void start() {
        setPathState(PathState.SCORE_PRELOAD);
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
//        やめてください.update();
        autonomousPathUpdate();

//        multipleTelemetry.addData("current state", やめてください.getCurrentState());
        multipleTelemetry.addData("path state", pathState);

        double frequency = 1 / period;
        double voltage = voltageSensor.getVoltage();

        multipleTelemetry.addData("frequency", frequency);
        multipleTelemetry.addData("voltage", voltage);

        multipleTelemetry.update();
    }

    public enum PathState {
        SCORE_PRELOAD,
        GRAB_PICKUP,
        PICK_SPECIMEN1,
        SCORE_SPECIMEN1,
        PICK_SPECIMEN2,
        SCORE_SPECIMEN2,
        PICK_SPECIMEN3,
        SCORE_SPECIMEN3,
        PICK_SPECIMEN4,
        SCORE_SPECIMEN4,
        PARK,
        TRANSITION_TO_TELEOPUS,
        AUTONOMOUS_FINISHED
    }
}