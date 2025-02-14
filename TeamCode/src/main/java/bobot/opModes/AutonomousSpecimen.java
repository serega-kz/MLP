package bobot.opModes;

import static bobot.controllers.YameteKudasai.*;
import static bobot.controllers.YameteKudasai.Alliance.*;
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

    private VoltageSensor voltageSensor;
    private MultipleTelemetry multipleTelemetry;

    private final Pose startPose = new Pose(7.900, 53.500);
    private final Pose scorePose = new Pose(50.000, 53.500);

    private final Pose pickup1CP1 = new Pose(0.000, 32.000);
    private final Pose pickup1CP2 = new Pose(72.000, 16.500);
    private final Pose pickup1Pose1 = new Pose(72.000, 2.500);
    private final Pose pickup1Pose2 = new Pose(20.000, 2.500);

    private final Pose pickup2CP = new Pose(72.000, 12.000);
    private final Pose pickup2Pose1 = new Pose(72.000, -9.500);
    private final Pose pickup2Pose2 = new Pose(20.000, -9.500);

    private final Pose pickup3CP = new Pose(72.000, 2.500);
    private final Pose pickup3Pose1 = new Pose(72.000, -16.500);
    private final Pose pickup3Pose2 = new Pose(20.000, -16.500);

    private final Pose specimen1CP = new Pose(28.000, 16.500);
    private final Pose specimenPose = new Pose(7.900, 16.500);

    private final Pose scoreCP1 = new Pose(42.000, 16.500);
    private final Pose scoreCP2 = new Pose(12.000, 53.500);

    private final Pose specimen2CP1 = new Pose(12.000, 53.500);
    private final Pose specimen2CP2 = new Pose(42.000, 16.500);

    private final Pose parkCP = new Pose(28.000, 60.000);
    private final Pose parkPose = new Pose(20.000, 8.000);

    private Path scorePreload, scoreSpecimen, grabSpecimen, park;
    private PathChain grabPickup;

    private PathState pathState;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(0);

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1CP1), new Point(pickup1CP2), new Point(pickup1Pose1)))
                .addPath(new BezierLine(new Point(pickup1Pose1), new Point(pickup1Pose2)))
                .addPath(new BezierCurve(new Point(pickup1Pose2), new Point(pickup2CP), new Point(pickup2Pose1)))
                .addPath(new BezierLine(new Point(pickup2Pose1), new Point(pickup2Pose2)))
                .addPath(new BezierCurve(new Point(pickup2Pose2), new Point(pickup3CP), new Point(pickup3Pose1)))
                .addPath(new BezierLine(new Point(pickup3Pose1), new Point(pickup3Pose2)))
                .addPath(new BezierCurve(new Point(pickup3Pose2), new Point(specimen1CP), new Point(specimenPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        scoreSpecimen = new Path(new BezierCurve(new Point(specimenPose), new Point(scoreCP1), new Point(scoreCP2), new Point(scorePose)));
        scoreSpecimen.setConstantHeadingInterpolation(0);

        grabSpecimen = new Path(new BezierCurve(new Point(scorePose), new Point(specimen2CP1), new Point(specimen2CP2), new Point(specimenPose)));
        grabSpecimen.setConstantHeadingInterpolation(0);

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkCP), new Point(parkPose)));
        park.setConstantHeadingInterpolation(0);
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private void autonomousPathUpdate() {
        if (pathState == SCORE_PRELOAD) {
            follower.followPath(scorePreload);
            setPathState(GRAB_PICKUP);
        } else if (pathState == GRAB_PICKUP) {
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            follower.followPath(grabPickup);
            setPathState(PICK_SPECIMEN1);
        } else if (pathState == PICK_SPECIMEN1 || pathState == PICK_SPECIMEN2 || pathState == PICK_SPECIMEN3 || pathState == PICK_SPECIMEN4) {
            if (isFollowerCooking()) return;

            if (やめてください.getCurrentState() == State.SPECIMEN_INTAKE) やめてください.proceedTransition();
            if (やめてください.getCurrentState() == State.SPECIMEN_OUTTAKE) setPathState(PathState.values()[pathState.ordinal() + 1]);
        } else if (pathState == SCORE_SPECIMEN1 || pathState == SCORE_SPECIMEN2 || pathState == SCORE_SPECIMEN3 || pathState == SCORE_SPECIMEN4) {
            if (isFollowerCooking()) return;

            follower.followPath(scoreSpecimen);
            setPathState(PathState.values()[pathState.ordinal() + 1]);
        } else if (pathState == GRAB_SPECIMEN1 || pathState == GRAB_SPECIMEN2 || pathState == GRAB_SPECIMEN3) {
            if (isFollowerCooking()) return;

            やめてください.proceedTransition();
            follower.followPath(grabSpecimen);
            setPathState(PathState.values()[pathState.ordinal() + 1]);
        } else if (pathState == PARK) {
            if (isFollowerCooking()) return;

            follower.followPath(park, true);
            setPathState(TRANSITION_TO_TELEOPUS);
        }
        else if (pathState == TRANSITION_TO_TELEOPUS) {
            if (isFollowerCooking()) return;

            やめてください.setScoringMode(ScoringMode.SAMPLE);
            setPathState(AUTONOMOUS_FINISHED);
        }
    }

    private void setPathState(PathState pathState) {
        this.pathState = pathState;
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        やめてください = new YameteKudasai(hardwareMap, NONE, YameteKudasai.OpMode.AUTONOMOUS_SPECIMEN);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

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
        GRAB_PICKUP,
        PICK_SPECIMEN1,
        SCORE_SPECIMEN1,
        GRAB_SPECIMEN1,
        PICK_SPECIMEN2,
        SCORE_SPECIMEN2,
        GRAB_SPECIMEN2,
        PICK_SPECIMEN3,
        SCORE_SPECIMEN3,
        GRAB_SPECIMEN3,
        PICK_SPECIMEN4,
        SCORE_SPECIMEN4,
        PARK,
        TRANSITION_TO_TELEOPUS,
        AUTONOMOUS_FINISHED
    }
}