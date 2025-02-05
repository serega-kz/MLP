package bobot.controllers;

import static bobot.controllers.ArmSubController.SampleColor.*;
import static bobot.controllers.YameteKudasai.OpMode.*;
import static bobot.controllers.YameteKudasai.ScoringMode.*;
import static bobot.controllers.YameteKudasai.State.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import bobot.Teleopus.Alliance;
import bobot.controllers.ArmSubController.ArmState;
import bobot.controllers.ArmSubController.SampleColor;
import bobot.controllers.PivotSubController.PivotState;
import bobot.controllers.SlideSubController.SlideState;

public class YameteKudasai {

    private final ArmSubController armSubController;
    private final PivotSubController pivotSubController;
    private final SlideSubController slideSubController;

    private final ElapsedTime transitionTime;
    private final SampleColor allianceColor;

    private State currentState, targetState;
    private ScoringMode scoringMode;

    private double wristRotationTimeout = 0;

    public YameteKudasai(HardwareMap hardwareMap, Alliance alliance, OpMode opMode) {
        armSubController = new ArmSubController(hardwareMap);
        pivotSubController = new PivotSubController(hardwareMap);
        slideSubController = new SlideSubController(hardwareMap);

        transitionTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        allianceColor = alliance == Alliance.RED ? RED : BLUE;

        if (opMode == AUTONOMOUS_SAMPLE) {
            armSubController.setTargetState(ArmState.SAMPLE_OUTTAKE1);
            targetState = AUTONOMOUS_SAMPLE_START;
            scoringMode = SAMPLE;
        } else if (opMode == AUTONOMOUS_SPECIMEN) {
            armSubController.setTargetState(ArmState.SPECIMEN_OUTTAKE);
            targetState = AUTONOMOUS_SPECIMEN_START;
            scoringMode = SPECIMEN;
        } else if (opMode == TELEOPUS) {
            targetState = TELEOPUS_START;
            scoringMode = SAMPLE;
        }
    }

    public void rotateWrist(Direction direction) {
        if (currentState != SAMPLE_INTAKE2) return;

        double deviation = (direction == Direction.CLOCKWISE ? ArmSubController.WRIST_POSITION_DEVIATION : -ArmSubController.WRIST_POSITION_DEVIATION);
        armSubController.deviateWristPosition(deviation);
    }

    public void deviateSlideTargetPosition(double deviation, double dt) {
        if (currentState != SAMPLE_INTAKE2 && currentState != SAMPLE_OUTTAKE && currentState != ASCENT2) return;
        slideSubController.deviateTargetPosition(deviation, dt);
    }

    public void setScoringMode(ScoringMode scoringMode) {
        if (this.scoringMode == scoringMode) return;

        if (scoringMode == SAMPLE && currentState != SPECIMEN_INTAKE && currentState != ASCENT1) return;
        if (scoringMode == SPECIMEN && currentState != SAMPLE_INTAKE1 && currentState != SAMPLE_INTAKE2 && currentState != ASCENT1) return;
        if (scoringMode == CYCLING && currentState != SAMPLE_INTAKE1 && currentState != SAMPLE_INTAKE2 && currentState != SPECIMEN_INTAKE) return;
        if (scoringMode == ASCENT && currentState != SAMPLE_INTAKE1 && currentState != SAMPLE_INTAKE2 && currentState != SPECIMEN_INTAKE) return;

        this.scoringMode = scoringMode;
        proceedTransition(true);
    }

    public void proceedTransition(boolean scoringModeJustChanged) {
        if (currentState != targetState) return;

        transitionTime.reset();
        int stateIndex = currentState.ordinal();

        if (scoringMode == SAMPLE && scoringModeJustChanged) {
            if (currentState == SAMPLE_INTAKE1 || currentState == SAMPLE_INTAKE2) return;

            currentState = TRANSITION_TO_SAMPLE;
            targetState = SAMPLE_INTAKE1;
        } else if (scoringMode == SAMPLE && stateIndex == SAMPLE_OUTTAKE.ordinal()) {
            targetState = SAMPLE_INTAKE1;
        } else if (scoringMode == SAMPLE) targetState = State.values()[stateIndex + 1];

        if (scoringMode == SPECIMEN && scoringModeJustChanged) {
            currentState = TRANSITION_TO_SPECIMEN;
            targetState = SPECIMEN_INTAKE;
        } else if (scoringMode == SPECIMEN && stateIndex == SPECIMEN_OUTTAKE.ordinal()) {
            targetState = SPECIMEN_INTAKE;
        } else if (scoringMode == SPECIMEN) targetState = State.values()[stateIndex + 1];

        if (scoringMode == CYCLING && scoringModeJustChanged) {
            if (currentState == SAMPLE_INTAKE1 || currentState == SAMPLE_INTAKE2) return;

            currentState = TRANSITION_TO_SAMPLE;
            targetState = SAMPLE_INTAKE1;
        } else if (scoringMode == CYCLING && stateIndex == CYCLING2.ordinal()) {
            targetState = SAMPLE_INTAKE1;
        } else if (scoringMode == CYCLING) targetState = State.values()[stateIndex + 1];

        if (scoringMode == ASCENT && scoringModeJustChanged) {
            currentState = TRANSITION_TO_ASCENT;
        } else if (scoringMode == ASCENT) targetState = ASCENT2;
    }

    private void completeOpModeTransition() {
        if (targetState == AUTONOMOUS_SAMPLE_START) {
            pivotSubController.setTargetPosition(PivotState.SAMPLE_OUTTAKE.targetPosition);
            slideSubController.setTargetPosition(SlideState.SAMPLE_OUTTAKE_HIGH.targetPosition);
            currentState = SAMPLE_OUTTAKE;
            targetState = SAMPLE_OUTTAKE;
        } else if (targetState == AUTONOMOUS_SPECIMEN_START) {
            pivotSubController.setTargetPosition(PivotState.SPECIMEN.targetPosition);
            slideSubController.setTargetPosition(SlideState.SPECIMEN_OUTTAKE.targetPosition);
            currentState = SPECIMEN_OUTTAKE;
            targetState = SPECIMEN_OUTTAKE;
        } else if (currentState == TELEOPUS_START) {
            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = TELEOPUS_START_1;
        } else if (targetState == TELEOPUS_START_1) {
            if (slideSubController.isCooking()) return;

            pivotSubController.setTargetPosition(PivotState.SAMPLE_INTAKE.targetPosition);
            currentState = SAMPLE_INTAKE1;
            targetState = SAMPLE_INTAKE1;
        }
    }

    private void completeSampleIntakeTransition() {
        if (currentState == SAMPLE_INTAKE1 && targetState == SAMPLE_INTAKE2) {
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE2.targetPosition);
            currentState = SAMPLE_INTAKE1_1;
        } else if (currentState == SAMPLE_INTAKE1_1) {
            if (transitionTime.time() <= 250) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE2.shoulderPosition);
            armSubController.setElbowPosition(ArmState.SAMPLE_INTAKE2.elbowPosition);
            currentState = SAMPLE_INTAKE1_2;
        } else if (currentState == SAMPLE_INTAKE1_2) {
            if (slideSubController.isCooking()) return;

            armSubController.setWristPosition(ArmState.SAMPLE_INTAKE2.wristPosition);
            currentState = SAMPLE_INTAKE2;
        } else if (currentState == SAMPLE_INTAKE2 && targetState == SAMPLE_INTAKE3) {
            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE3.shoulderPosition);
            armSubController.setSensorLED(true);
            currentState = SAMPLE_INTAKE2_1;
        } else if (currentState == SAMPLE_INTAKE2_1) {
            if (transitionTime.time() <= 200) return;
            transitionTime.reset();

            armSubController.setClawPosition(ArmState.SAMPLE_INTAKE3.clawPosition);
            currentState = SAMPLE_INTAKE2_2;
        } else if (currentState == SAMPLE_INTAKE2_2) {
            if (transitionTime.time() <= 300) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE2.shoulderPosition);
            currentState = SAMPLE_INTAKE2_3;
        } else if (currentState == SAMPLE_INTAKE2_3) {
            if (transitionTime.time() <= 500) return;
            transitionTime.reset();

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor == YELLOW || sampleColor == allianceColor) {
                armSubController.setWristPosition(ArmState.SAMPLE_INTAKE1.wristPosition);
                wristRotationTimeout = 75 * armSubController.getWristDeviationCount();
                armSubController.setSensorLED(false);

                currentState = SAMPLE_INTAKE3;
                if (scoringMode == SAMPLE || sampleColor == YELLOW) targetState = SAMPLE_OUTTAKE;
                if (scoringMode == CYCLING) targetState = CYCLING1;
            } else {
                armSubController.setClawPosition(ArmState.SAMPLE_INTAKE2.clawPosition);

                currentState = SAMPLE_INTAKE2;
                targetState = SAMPLE_INTAKE2;
            }
        }
    }

    private void completeSampleTransition() {
        completeSampleIntakeTransition();

        if (currentState == SAMPLE_INTAKE3 && targetState == SAMPLE_OUTTAKE) {
            if (transitionTime.time() <= wristRotationTimeout) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_OUTTAKE1.shoulderPosition);
            armSubController.setElbowPosition(ArmState.SAMPLE_OUTTAKE1.elbowPosition);
            slideSubController.setTargetPosition(SlideState.SAMPLE_OUTTAKE.targetPosition);
            currentState = SAMPLE_INTAKE3_1;
        } else if (currentState == SAMPLE_INTAKE3_1) {
            if (slideSubController.isCooking()) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_OUTTAKE2.shoulderPosition);
            armSubController.setElbowPosition(ArmState.SAMPLE_OUTTAKE2.elbowPosition);
            pivotSubController.setTargetPosition(PivotState.SAMPLE_OUTTAKE.targetPosition);
            currentState = SAMPLE_INTAKE3_2;
        } else if (currentState == SAMPLE_INTAKE3_2) {
             if (pivotSubController.isCooking()) return;

            armSubController.setWristPosition(armSubController.getWristScoringPosition());
            slideSubController.setTargetPosition(slideSubController.getScoringPosition());
            currentState = SAMPLE_OUTTAKE;
        } else if (currentState == SAMPLE_OUTTAKE && targetState == SAMPLE_INTAKE1) {
            armSubController.setClawPosition(ArmState.SAMPLE_OUTTAKE2.clawPosition);
            slideSubController.updateScoringPosition();
            currentState = SAMPLE_OUTTAKE_1;
        } else if (currentState == SAMPLE_OUTTAKE_1) {
            if (transitionTime.time() <= 100) return;

            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            currentState = SAMPLE_OUTTAKE_2;
        } else if (currentState == SAMPLE_OUTTAKE_2) {
            if (transitionTime.time() <= 300) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = SAMPLE_OUTTAKE_3;
        } else if (currentState == SAMPLE_OUTTAKE_3) {
            if (slideSubController.isCooking()) return;

            pivotSubController.setTargetPosition(PivotState.SAMPLE_INTAKE.targetPosition);
            currentState = SAMPLE_INTAKE1;
        }
    }

    private void completeSpecimenTransition() {
        if (currentState == SPECIMEN_INTAKE && targetState == SPECIMEN_OUTTAKE) {
            armSubController.setShoulderPosition(ArmState.SPECIMEN_INTAKE2.shoulderPosition);
            currentState = SPECIMEN_INTAKE_1;
        } else if (currentState == SPECIMEN_INTAKE_1) {
            if (transitionTime.time() <= 200) return;

            armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE2.clawPosition);
            currentState = SPECIMEN_INTAKE_2;
        } else if (currentState == SPECIMEN_INTAKE_2) {
            if (transitionTime.time() <= 300) return;

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor != NONE) {
                armSubController.setSensorLED(false);
                armSubController.setShoulderPosition(ArmState.SPECIMEN_OUTTAKE.shoulderPosition);
                slideSubController.setTargetPosition(SlideState.SPECIMEN_OUTTAKE.targetPosition);
                currentState = SPECIMEN_INTAKE_3;
            } else {
                armSubController.setShoulderPosition(ArmState.SPECIMEN_INTAKE1.shoulderPosition);
                armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE1.clawPosition);

                currentState = SPECIMEN_INTAKE;
                targetState = SPECIMEN_INTAKE;
            }
        } else if (currentState == SPECIMEN_INTAKE_3) {
            if (transitionTime.time() <= 700) return;

            armSubController.setElbowPosition(ArmState.SPECIMEN_OUTTAKE.elbowPosition);
            armSubController.setWristPosition(ArmState.SPECIMEN_OUTTAKE.wristPosition);
            currentState = SPECIMEN_OUTTAKE;
        } else if (currentState == SPECIMEN_OUTTAKE && targetState == SPECIMEN_INTAKE) {
            armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE1.clawPosition);
            currentState = SPECIMEN_OUTTAKE_1;
        } else if (currentState == SPECIMEN_OUTTAKE_1) {
            if (transitionTime.time() <= 100) return;

            armSubController.setElbowPosition(ArmState.SPECIMEN_INTAKE1.elbowPosition);
            armSubController.setWristPosition(ArmState.SPECIMEN_INTAKE1.wristPosition);
            currentState = SPECIMEN_OUTTAKE_2;
        } else if (currentState == SPECIMEN_OUTTAKE_2) {
            if (transitionTime.time() <= 500) return;

            armSubController.setShoulderPosition(ArmState.SPECIMEN_INTAKE1.shoulderPosition);
            armSubController.setSensorLED(true);
            slideSubController.setTargetPosition(SlideState.SPECIMEN_INTAKE.targetPosition);
            currentState = SPECIMEN_INTAKE;
        }
    }

    private void completeCyclingTransition() {
        completeSampleIntakeTransition();

        if (currentState == SAMPLE_INTAKE3 && targetState == CYCLING1) {
            if (transitionTime.time() <= wristRotationTimeout) return;

            armSubController.setElbowPosition(ArmState.SAMPLE_INTAKE1.elbowPosition);
            slideSubController.setTargetPosition(SlideState.CYCLING1.targetPosition);
            currentState = CYCLING1;
        } else if (currentState == CYCLING1 && targetState == CYCLING2) {
            slideSubController.setTargetPosition(SlideState.CYCLING2.targetPosition);
            currentState = CYCLING2;
        } else if (currentState == CYCLING2 && targetState == SAMPLE_INTAKE1) {
            armSubController.setClawPosition(ArmState.SAMPLE_INTAKE1.clawPosition);
            currentState = CYCLING2_1;
        } else if (currentState == CYCLING2_1) {
            if (transitionTime.time() <= 100) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = SAMPLE_INTAKE1;
        }
    }

    private void completeAscentTransition() {
        if (currentState == ASCENT1 && targetState == ASCENT2) {
            pivotSubController.setTargetPosition(PivotState.ASCENT2.targetPosition);
            slideSubController.setTargetPosition(SlideState.ASCENT1_1.targetPosition);
            currentState = ASCENT1_1;
        } else if (currentState == ASCENT1_1) {
            if (slideSubController.isCooking()) return;

            slideSubController.setTargetPosition(SlideState.ASCENT2.targetPosition);
            currentState = ASCENT2;
        }
    }

    private void completeStateTransition() {
        if (currentState == TRANSITION_TO_SAMPLE) {
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = TRANSITION_TO_SAMPLE_1;
        } else if (currentState == TRANSITION_TO_SAMPLE_1) {
            if (slideSubController.isCooking()) return;

            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            pivotSubController.setTargetPosition(PivotState.SAMPLE_INTAKE.targetPosition);
            currentState = SAMPLE_INTAKE1;
        } else if (currentState == TRANSITION_TO_SPECIMEN) {
            slideSubController.setTargetPosition(SlideState.SPECIMEN_INTAKE.targetPosition);
            currentState = TRANSITION_TO_SPECIMEN_1;
        } else if (currentState == TRANSITION_TO_SPECIMEN_1) {
            if (slideSubController.isCooking()) return;

            armSubController.setTargetState(ArmState.SPECIMEN_INTAKE1);
            armSubController.setSensorLED(true);
            pivotSubController.setTargetPosition(PivotState.SPECIMEN.targetPosition);
            currentState = SPECIMEN_INTAKE;
        } else if (currentState == TRANSITION_TO_ASCENT) {
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = TRANSITION_TO_ASCENT_1;
        } else if (currentState == TRANSITION_TO_ASCENT_1) {
            if (slideSubController.isCooking()) return;

            armSubController.setTargetState(ArmState.ASCENT);
            pivotSubController.setTargetPosition(PivotState.ASCENT1.targetPosition);
            currentState = TRANSITION_TO_ASCENT_2;
        } else if (currentState == TRANSITION_TO_ASCENT_2) {
            if (pivotSubController.isCooking()) return;

            slideSubController.setTargetPosition(SlideState.ASCENT1.targetPosition);
            currentState = ASCENT1;
        }
    }

    public void update() {
        pivotSubController.update();
        slideSubController.update();

        if (currentState == targetState) return;

        if (targetState.ordinal() <= 2) completeOpModeTransition();
        else if (scoringMode == SAMPLE) completeSampleTransition();
        else if (scoringMode == SPECIMEN) completeSpecimenTransition();
        else if (scoringMode == CYCLING) completeCyclingTransition();
        else if (scoringMode == ASCENT) completeAscentTransition();
        else completeStateTransition();
    }

    public State getCurrentState() {
        return currentState;
    }

    public enum OpMode {AUTONOMOUS_SAMPLE, AUTONOMOUS_SPECIMEN, TELEOPUS}

    public enum Direction {ANTICLOCKWISE, CLOCKWISE}

    public enum ScoringMode {SAMPLE, CYCLING, SPECIMEN, ASCENT}

    public enum State {
        AUTONOMOUS_SAMPLE_START,
        AUTONOMOUS_SPECIMEN_START,
        TELEOPUS_START,
        TELEOPUS_START_1,
        SAMPLE_INTAKE1,
        SAMPLE_INTAKE2,
        SAMPLE_INTAKE3,
        SAMPLE_OUTTAKE,
        SAMPLE_INTAKE1_1,
        SAMPLE_INTAKE1_2,
        SAMPLE_INTAKE2_1,
        SAMPLE_INTAKE2_2,
        SAMPLE_INTAKE2_3,
        SAMPLE_INTAKE2_4,
        SAMPLE_INTAKE3_1,
        SAMPLE_INTAKE3_2,
        SAMPLE_OUTTAKE_1,
        SAMPLE_OUTTAKE_2,
        SAMPLE_OUTTAKE_3,
        SPECIMEN_INTAKE,
        SPECIMEN_OUTTAKE,
        SPECIMEN_INTAKE_1,
        SPECIMEN_INTAKE_2,
        SPECIMEN_INTAKE_3,
        SPECIMEN_OUTTAKE_1,
        SPECIMEN_OUTTAKE_2,
        CYCLING1,
        CYCLING2,
        CYCLING2_1,
        ASCENT1,
        ASCENT2,
        ASCENT1_1,
        TRANSITION_TO_SAMPLE,
        TRANSITION_TO_SAMPLE_1,
        TRANSITION_TO_SPECIMEN,
        TRANSITION_TO_SPECIMEN_1,
        TRANSITION_TO_ASCENT,
        TRANSITION_TO_ASCENT_1,
        TRANSITION_TO_ASCENT_2
    }
}
