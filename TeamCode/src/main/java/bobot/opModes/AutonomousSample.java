package bobot.opModes;

import static java.lang.Math.*;
import static bobot.controllers.YameteKudasai.Alliance.*;
import static bobot.controllers.YameteKudasai.State.*;

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

    private final Pose startPose = new Pose(7.9, 113.5, -PI / 2);
    private final Pose scorePose = new Pose(13.0, 132.0, -PI / 4);

    private final Pose score1CP = new Pose(24.0, 120.0);

    private final Pose pickup1Pose = new Pose(24.0, 123.0, 0);
    private final Pose pickup2Pose = new Pose(24.0, 133.0, 0);

    private final Pose pickup3CP = new Pose(48.0, 108.0);
    private final Pose pickup3Pose = new Pose(46.5, 135.0, -PI / 2);
    private final Pose score4CP = new Pose(48.0, 108.0);

    private final Pose parkCP = new Pose(72.0, 120.0);
    private final Pose parkPose1 = new Pose(60.0, 110.0, -PI / 2);
    private final Pose parkPose2 = new Pose(60.0, 100.0, -PI / 2);

    private Path scorePreload, park1, park2;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private int pathState;

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

        park1 = new Path(new BezierCurve(new Point(scorePose), new Point(parkCP), new Point(parkPose1)));
        park1.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose1.getHeading());

        park2 = new Path(new BezierLine(new Point(parkPose1), new Point(parkPose2)));
        park2.setConstantHeadingInterpolation(parkPose1.getHeading());
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.setMaxPower(0.6);
            follower.followPath(scorePreload);
            setPathState(1);
        } else if (pathState == 1) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(2);
        } else if (pathState == 2) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.setMaxPower(0.4);
            follower.followPath(grabPickup1);
            setPathState(3);
        } else if (pathState == 3) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE1) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(4);
        } else if (pathState == 4) {
            if (pathTimer.getElapsedTime() <= 750) return;
            if (やめてください.getCurrentState() != SAMPLE_INTAKE2) return;

            やめてください.proceedTransition();
            setPathState(5);
        } else if (pathState == 5) {
            if (pathTimer.getElapsedTime() <= 1000) return;
            if (やめてください.getCurrentState() != SAMPLE_INTAKE3) return;

            follower.followPath(scorePickup1);
            setPathState(6);
        } else if (pathState == 6) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(7);
        } else if (pathState == 7) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.followPath(grabPickup2);
            setPathState(8);
        } else if (pathState == 8) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE1) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(9);
        } else if (pathState == 9) {
            if (pathTimer.getElapsedTime() <= 750) return;
            if (やめてください.getCurrentState() != SAMPLE_INTAKE2) return;

            やめてください.proceedTransition();
            setPathState(10);
        } else if (pathState == 10) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE3) return;
            if (pathTimer.getElapsedTime() <= 1000) return;

            follower.setMaxPower(0.4);
            follower.followPath(scorePickup2);
            setPathState(11);
        } else if (pathState == 11) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedAutoTransition();
            setPathState(12);
        } else if (pathState == 12) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_AUTO_2) return;

            follower.setMaxPower(0.6);
            follower.followPath(grabPickup3);
            setPathState(13);
        } else if (pathState == 13) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE_AUTO) return;
            if (isFollowerCooking()) return;

            やめてください.proceedAutoTransition();
            setPathState(14);
        } else if (pathState == 14) {
            if (やめてください.getCurrentState() != SAMPLE_INTAKE_AUTO_3) return;

            follower.followPath(scorePickup3);
            setPathState(15);
        } else if (pathState == 15) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE) return;
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            setPathState(16);
        } else if (pathState == 16) {
            if (やめてください.getCurrentState() != SAMPLE_OUTTAKE_2) return;

            follower.setMaxPower(1.0);
            follower.followPath(park1);
            setPathState(17);
        } else if (pathState == 17) {
            if (isFollowerCooking()) return;

            やめてください.setPivotTargetPosition(680);
            やめてください.setSlideTargetPosition(800);
            pathState = 18;
        } else if (pathState == 18) {
            if (pathTimer.getElapsedTime() <= 5000) return;

            follower.followPath(park2);
            pathState = 19;
        } else if (pathState == 19) {
            if (isFollowerCooking()) return;
            pathState = 20;
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

        やめてください = new YameteKudasai(hardwareMap, NONE, YameteKudasai.OpMode.AUTONOMOUS_SAMPLE);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.3);

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
        setPathState(0);
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
}
