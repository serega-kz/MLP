package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "FLMotor";
        FollowerConstants.leftRearMotorName = "BLMotor";
        FollowerConstants.rightFrontMotorName = "FRMotor";
        FollowerConstants.rightRearMotorName = "BRMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 11.7;

        FollowerConstants.xMovement = 58;
        FollowerConstants.yMovement = 42;

        FollowerConstants.forwardZeroPowerAcceleration = -54;
        FollowerConstants.lateralZeroPowerAcceleration = -75;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15, 0, 0.02, 0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.2, 0, 0.12, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0015, 0, 0, 0.0004, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 10;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
