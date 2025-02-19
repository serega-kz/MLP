package bobot.opModes;

import static java.lang.Math.PI;
import static bobot.controllers.YameteKudasai.Alliance.NONE;
import static bobot.controllers.YameteKudasai.State.SPECIMEN_INTAKE;
import static bobot.controllers.YameteKudasai.State.SPECIMEN_INTAKE_5;

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

import bobot.controllers.SlideSubController;
import bobot.controllers.YameteKudasai;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "!0autonomous")
public class AutonomousSpecimen extends OpMode {

    private List<LynxModule> allHubs;
    private double lastTimeStamp;

    private Follower follower;
    private YameteKudasai やめてください;

    private Timer pathTimer;

    private VoltageSensor voltageSensor;
    private MultipleTelemetry multipleTelemetry;

    private final Pose startPose = new Pose(7.9, 65.6, PI);
    private final Pose scorePose = new Pose(37.0, 65.6, PI);

    private final Pose pickup0CP = new Pose(32.0, 56.0, PI);
    private final Pose pickup0Pose = new Pose(32.0, 36.0, PI);

    private final Pose pickup1CP = new Pose(46.0, 38.0, PI);
    private final Pose pickup1Pose1 = new Pose(56.0, 22.5, PI);
    private final Pose pickup1Pose2 = new Pose(20.0, 22.5, PI);

    private final Pose pickup2CP = new Pose(46.0, 28.0, PI);
    private final Pose pickup2Pose1 = new Pose(56.0, 12.5, PI);
    private final Pose pickup2Pose2 = new Pose(20.0, 12.5, PI);

    private final Pose pickup3CP = new Pose(46.0, 17.0, PI);
    private final Pose pickup3Pose1 = new Pose(56.0, 7.0, PI);
    private final Pose pickup3Pose2 = new Pose(20.0, 7.0, PI);

    private final Pose specimenCP = new Pose(20.0, 72.0, PI);
    private final Pose specimenGrabPose = new Pose(20.0, 32.0, PI);
    private final Pose specimenScorePose1 = new Pose(38.0, 67.6, PI);
    private final Pose specimenScorePose2 = new Pose(38.0, 69.6, PI);
    private final Pose specimenScorePose3 = new Pose(38.0, 71.6, PI);

    private final Pose parkPose = new Pose(18.0, 30.0, 0);

    private Path scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;

    private Path grabSpecimen;
    private PathChain scoreSpecimen1, scoreSpecimen2, scoreSpecimen3;

    private int pathState;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup0CP), new Point(pickup0Pose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierCurve(new Point(pickup0Pose), new Point(pickup1CP), new Point(pickup1Pose1)))
                .setConstantHeadingInterpolation(pickup0Pose.getHeading())
                .addPath(new BezierLine(new Point(pickup1Pose1), new Point(pickup1Pose2)))
                .setConstantHeadingInterpolation(pickup1Pose1.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1Pose2), new Point(pickup2CP), new Point(pickup2Pose1)))
                .setConstantHeadingInterpolation(pickup1Pose2.getHeading())
                .addPath(new BezierLine(new Point(pickup2Pose1), new Point(pickup2Pose2)))
                .setConstantHeadingInterpolation(pickup2Pose1.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup2Pose2), new Point(pickup3CP), new Point(pickup3Pose1)))
                .setConstantHeadingInterpolation(pickup2Pose2.getHeading())
                .addPath(new BezierLine(new Point(pickup3Pose1), new Point(pickup3Pose2)))
                .setConstantHeadingInterpolation(pickup3Pose1.getHeading())
                .build();

        grabSpecimen = new Path(new BezierLine(new Point(pickup3Pose1), new Point(specimenGrabPose)));
        grabSpecimen.setConstantHeadingInterpolation(pickup3Pose1.getHeading());

        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenGrabPose), new Point(specimenCP), new Point(specimenScorePose1)))
                .setConstantHeadingInterpolation(specimenGrabPose.getHeading())
                .addPath(new BezierCurve(new Point(specimenScorePose1), new Point(specimenCP), new Point(specimenGrabPose)))
                .setConstantHeadingInterpolation(specimenScorePose1.getHeading())
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenGrabPose), new Point(specimenCP), new Point(specimenScorePose2)))
                .setConstantHeadingInterpolation(specimenGrabPose.getHeading())
                .addPath(new BezierCurve(new Point(specimenScorePose2), new Point(specimenCP), new Point(specimenGrabPose)))
                .setConstantHeadingInterpolation(specimenScorePose2.getHeading())
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenGrabPose), new Point(specimenCP), new Point(specimenScorePose3)))
                .setConstantHeadingInterpolation(specimenGrabPose.getHeading())
                .addPath(new BezierCurve(new Point(specimenScorePose3), new Point(specimenCP), new Point(parkPose)))
                .setLinearHeadingInterpolation(specimenScorePose3.getHeading(), parkPose.getHeading())
                .build();
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private void autonomousPathUpdate() {
        if (pathState == -1) {
            if (pathTimer.getElapsedTime() <= 800) return;
            setPathState(0);
        } else if (pathState == 0) {
            follower.followPath(scorePreload, false);
            setPathState(1);
        } else if (pathState == 1) {
            if (isFollowerCooking()) return;

            follower.followPath(grabPickup1, false);
            やめてください.proceedTransition();
            setPathState(2);
        } else if (pathState == 2) {
            if (isFollowerCooking()) return;

            follower.followPath(grabPickup2, false);
            setPathState(3);
        } else if (pathState == 3) {

            if (isFollowerCooking()) return;

            follower.followPath(grabPickup3, false);
            setPathState(4);
        } else if (pathState == 4) {
            if (isFollowerCooking()) return;

            follower.followPath(grabSpecimen);
            setPathState(5);
        } else if (pathState == 5) {
            if (isFollowerCooking()) return;

            if (やめてください.getCurrentState() == SPECIMEN_INTAKE) やめてください.proceedTransition();
            else if (やめてください.getCurrentState() == SPECIMEN_INTAKE_5) {
                follower.followPath(scoreSpecimen1, true);
                setPathState(6);
            }
        }
        else if (pathState == 6) {
            if (follower.getPose().getX() >= 38.5) やめてください.proceedTransition();
            if (isFollowerCooking()) return;

            if (やめてください.getCurrentState() == SPECIMEN_INTAKE) やめてください.proceedTransition();
            else if (やめてください.getCurrentState() == SPECIMEN_INTAKE_5) {
                follower.followPath(scoreSpecimen2, true);
                setPathState(7);
            }
        } else if (pathState == 7) {
            if (follower.getPose().getX() >= 38.5) やめてください.proceedTransition();
            if (isFollowerCooking()) return;

            if (やめてください.getCurrentState() == SPECIMEN_INTAKE) やめてください.proceedTransition();
            else if (やめてください.getCurrentState() == SPECIMEN_INTAKE_5) {
                follower.followPath(scoreSpecimen3, true);
                setPathState(8);
            }
        } else if (pathState == 8) {
            if (follower.getPose().getX() >= 38.5) やめてください.proceedTransition();
            if (isFollowerCooking()) return;

            やめてください.setSlideTargetPosition(SlideSubController.SlideState.ZERO.targetPosition);
            setPathState(-2);
        }
    }

    private void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        やめてください = new YameteKudasai(hardwareMap, NONE, YameteKudasai.OpMode.AUTONOMOUS_SPECIMEN);

        pathTimer = new Timer();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void start() {
        setPathState(-1);
        lastTimeStamp = 0;
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        やめてください.update();
        autonomousPathUpdate();

        follower.update();

        multipleTelemetry.addData("current state", やめてください.getCurrentState());
        multipleTelemetry.addData("current t value", follower.getCurrentTValue());
        multipleTelemetry.addData("path state", pathState);

        double frequency = 1 / period;
        double voltage = voltageSensor.getVoltage();

        multipleTelemetry.addData("frequency", frequency);
        multipleTelemetry.addData("voltage", voltage);

        multipleTelemetry.update();
    }
}