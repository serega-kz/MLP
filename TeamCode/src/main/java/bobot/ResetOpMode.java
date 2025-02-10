package bobot;

import static bobot.controllers.ArmSubController.*;
import static bobot.controllers.YameteKudasai.*;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import bobot.controllers.ArmSubController;
import bobot.controllers.PivotSubController;

public class ResetOpMode extends LinearOpMode {

    protected OpMode opMode;

    @Override
    public void runOpMode() {
        ArmSubController armSubController = new ArmSubController(hardwareMap);

        Motor pivotMotor1 = new Motor(hardwareMap, "pivotMotor1");
        Motor pivotMotor2 = new Motor(hardwareMap, "pivotMotor2");
        PivotSubController pivotSubController = new PivotSubController(hardwareMap);

        Motor slideMotor1 = new Motor(hardwareMap, "slideMotor1");
        Motor slideMotor2 = new Motor(hardwareMap, "slideMotor2");
        slideMotor2.setInverted(true);

        ElapsedTime transitionTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();

        transitionTime.reset();
        while (opModeIsActive()) {
            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);

            while (transitionTime.time() <= 500) {}

            slideMotor1.set(-0.80);
            slideMotor2.set(-0.80);

            double slideMotor1Current = 0;
            double slideMotor2Current = 0;
            while (slideMotor1Current <= 3 && slideMotor2Current <= 3) {
                slideMotor1Current = ((DcMotorEx) slideMotor1.motor).getCurrent(CurrentUnit.AMPS);
                slideMotor2Current = ((DcMotorEx) slideMotor2.motor).getCurrent(CurrentUnit.AMPS);
            }

            slideMotor1.set(0.00);
            slideMotor2.set(0.00);

            pivotMotor1.set(0.80);
            pivotMotor2.set(0.80);

            double pivotMotor1Current = 0;
            double pivotMotor2Current = 0;
            while (pivotMotor1Current <= 3 && pivotMotor2Current <= 3) {
                pivotMotor1Current = ((DcMotorEx) pivotMotor1.motor).getCurrent(CurrentUnit.AMPS);
                pivotMotor2Current = ((DcMotorEx) pivotMotor2.motor).getCurrent(CurrentUnit.AMPS);
            }

            if (opMode == OpMode.AUTONOMOUS_SAMPLE) pivotSubController.setTargetPosition(PivotSubController.PivotState.SAMPLE_OUTTAKE.targetPosition);
            else if (opMode == OpMode.AUTONOMOUS_SPECIMEN) pivotSubController.setTargetPosition(PivotSubController.PivotState.SPECIMEN.targetPosition);
            else if (opMode == OpMode.TELEOPUS) pivotSubController.setTargetPosition(PivotSubController.PivotState.SAMPLE_INTAKE.targetPosition);

            while (pivotSubController.isCooking()) pivotSubController.update();

            if (opMode == OpMode.AUTONOMOUS_SAMPLE) armSubController.setTargetState(ArmState.SAMPLE_OUTTAKE1);
            else if (opMode == OpMode.AUTONOMOUS_SPECIMEN) armSubController.setTargetState(ArmState.SPECIMEN_OUTTAKE);
            else if (opMode == OpMode.TELEOPUS) armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);

            transitionTime.reset();
            while (pivotSubController.isCooking()) pivotSubController.update();

            slideMotor1.stopAndResetEncoder();
            slideMotor2.stopAndResetEncoder();

            requestOpModeStop();
        }
    }
}
