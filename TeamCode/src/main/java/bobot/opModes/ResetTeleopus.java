package bobot.opModes;

import static bobot.controllers.ArmSubController.ArmState;
import static bobot.opModes.ResetTeleopus.ResetState.PIVOT_PRESET1;
import static bobot.opModes.ResetTeleopus.ResetState.PIVOT_RESET;
import static bobot.opModes.ResetTeleopus.ResetState.RESET_FINISHED;
import static bobot.opModes.ResetTeleopus.ResetState.SLIDE_RESET;
import static bobot.opModes.ResetTeleopus.ResetState.START;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import bobot.controllers.ArmSubController;

@TeleOp(group = "!0teleopus")
public class ResetTeleopus extends LinearOpMode {

    private Motor pivotMotor1, pivotMotor2;
    private Motor slideMotor1, slideMotor2;

    private void setPivotMotors(double power) {
        pivotMotor1.set(power);
        pivotMotor2.set(power);
    }

    private void setSlideMotors(double power) {
        slideMotor1.set(power);
        slideMotor2.set(power);
    }

    @Override
    public void runOpMode() {
        ArmSubController armSubController = new ArmSubController(hardwareMap);
        armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);

        pivotMotor1 = new Motor(hardwareMap, "pivotMotor1");
        pivotMotor2 = new Motor(hardwareMap, "pivotMotor2");

        slideMotor1 = new Motor(hardwareMap, "slideMotor1");
        slideMotor2 = new Motor(hardwareMap, "slideMotor2");
        slideMotor2.setInverted(true);

        ElapsedTime transitionTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        ResetState currentState = START;
        while (opModeIsActive()) {

            double slideMotor1Current = ((DcMotorEx) slideMotor1.motor).getCurrent(CurrentUnit.AMPS);
            double slideMotor2Current = ((DcMotorEx) slideMotor2.motor).getCurrent(CurrentUnit.AMPS);
            double pivotMotor1Current = ((DcMotorEx) pivotMotor1.motor).getCurrent(CurrentUnit.AMPS);
            double pivotMotor2Current = ((DcMotorEx) pivotMotor2.motor).getCurrent(CurrentUnit.AMPS);

            telemetry.addData("current state", currentState);
            telemetry.addData("pivot motor1 current", pivotMotor1Current);
            telemetry.addData("pivot motor1 current", pivotMotor2Current);
            telemetry.addData("slide motor1 current", slideMotor1Current);
            telemetry.addData("slide motor2 current", slideMotor2Current);
            telemetry.update();

            if (currentState == START) {
                setSlideMotors(-0.60);
                currentState = SLIDE_RESET;
            } else if (currentState == SLIDE_RESET) {
                if (slideMotor1Current <= 3 || slideMotor2Current <= 3) continue;

                setSlideMotors(0.00);
                setPivotMotors(-0.60);
                currentState = PIVOT_RESET;
            } else if (currentState == PIVOT_RESET) {
                if (pivotMotor1Current <= 3 || pivotMotor2Current <= 3) continue;

                setPivotMotors(0.00);
                transitionTime.reset();
                currentState = PIVOT_PRESET1;
            } else if (currentState == PIVOT_PRESET1) {
                if (transitionTime.time() <= 200) continue;

                pivotMotor1.stopAndResetEncoder();
                pivotMotor2.stopAndResetEncoder();
                slideMotor1.stopAndResetEncoder();
                slideMotor2.stopAndResetEncoder();
                currentState = RESET_FINISHED;
            } else {
                requestOpModeStop();
            }
        }
    }

    public enum ResetState {START, SLIDE_RESET, PIVOT_RESET, PIVOT_PRESET1, RESET_FINISHED}
}
