package bobot.controllers;

import static bobot.controllers.ArmSubController.SampleColor.*;
import static bobot.controllers.YameteKudasai.OpMode.*;
import static bobot.controllers.YameteKudasai.ScoringMode.*;
import static bobot.controllers.YameteKudasai.State.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import bobot.controllers.ArmSubController.ArmState;
import bobot.controllers.ArmSubController.SampleColor;
import bobot.controllers.PivotSubController.PivotState;
import bobot.controllers.SlideSubController.SlideState;

public class YameteKudasai {

    private final ArmSubController armSubController;
    private final PivotSubController pivotSubController;
    private final SlideSubController slideSubController;

    private final ElapsedTime transitionTimer;
    private final SampleColor allianceColor;

    private State currentState, targetState;
    private ScoringMode scoringMode;

    private boolean isAutonomous = false;

    public YameteKudasai(HardwareMap hardwareMap, Alliance alliance, OpMode opMode) {
        armSubController = new ArmSubController(hardwareMap);
        pivotSubController = new PivotSubController(hardwareMap);
        slideSubController = new SlideSubController(hardwareMap);

        transitionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        allianceColor = alliance == Alliance.RED ? RED : BLUE;
        if (opMode == AUTONOMOUS_SAMPLE) {
            armSubController.setTargetState(ArmState.SAMPLE_OUTTAKE2);
            targetState = AUTONOMOUS_SAMPLE_START;
            scoringMode = SAMPLE;
            isAutonomous = true;
        } else if (opMode == AUTONOMOUS_SPECIMEN) {
            armSubController.setTargetState(ArmState.SPECIMEN_INTAKE2);
            targetState = AUTONOMOUS_SPECIMEN_START;
            scoringMode = SPECIMEN;
            isAutonomous = true;
        } else if (opMode == TELEOPUS) {
            targetState = TELEOPUS_START;
            scoringMode = SAMPLE;
        }
    }

    public void rotateWrist(Direction direction) {
        if (currentState != SAMPLE_INTAKE2 && currentState != SAMPLE_OUTTAKE) return;

        double deviation = direction == Direction.CLOCKWISE ? -0.140 : 0.140;
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

    public void proceedAutoTransition() {
        transitionTimer.reset();
        if (currentState == SAMPLE_OUTTAKE) targetState = SAMPLE_INTAKE_AUTO;
        else if (currentState == SAMPLE_INTAKE_AUTO) targetState = SAMPLE_OUTTAKE_AUTO;
    }

    public void proceedTransition() {
        proceedTransition(false);
    }

    public void proceedTransition(boolean scoringModeJustChanged) {
        if (currentState != targetState) return;

        transitionTimer.reset();
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
            targetState = ASCENT1;
        } else if (scoringMode == ASCENT) targetState = ASCENT2;
    }

    private void completeOpModeTransition() {
        if (targetState == AUTONOMOUS_SAMPLE_START) {
            pivotSubController.setTargetPosition(PivotState.SAMPLE_OUTTAKE.targetPosition);
            targetState = AUTONOMOUS_SAMPLE_START_1;
        } else if (targetState == AUTONOMOUS_SAMPLE_START_1) {
            if (pivotSubController.isCooking()) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_OUTTAKE_HIGH.targetPosition);
            currentState = SAMPLE_OUTTAKE;
            targetState = SAMPLE_OUTTAKE;
        } else if (targetState == AUTONOMOUS_SPECIMEN_START) {
            pivotSubController.setTargetPosition(PivotState.SPECIMEN.targetPosition);
            targetState = AUTONOMOUS_SPECIMEN_START_1;
        } else if (targetState == AUTONOMOUS_SPECIMEN_START_1) {
            armSubController.setTargetState(ArmState.SPECIMEN_OUTTAKE);
            slideSubController.setTargetPosition(SlideState.SPECIMEN_OUTTAKE.targetPosition);
            currentState = SPECIMEN_OUTTAKE;
            targetState = SPECIMEN_OUTTAKE;
        } else if (targetState == TELEOPUS_START) {
            transitionTimer.reset();
            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            targetState = TELEOPUS_START_1;
        } else if (targetState == TELEOPUS_START_1) {
            if (transitionTimer.time() <= 750) return;

            pivotSubController.setTargetPosition(PivotState.SAMPLE_INTAKE.targetPosition);
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = SAMPLE_INTAKE1;
            targetState = SAMPLE_INTAKE1;
        }
    }

    private void completeSampleIntakeTransition() {
        if (currentState == SAMPLE_INTAKE1 && targetState == SAMPLE_INTAKE2) {
            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE2.targetPosition);
            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE2.shoulderPosition);
            armSubController.setElbowPosition(ArmState.SAMPLE_INTAKE2.elbowPosition);
            currentState = SAMPLE_INTAKE2;
        } else if (currentState == SAMPLE_INTAKE2 && targetState == SAMPLE_INTAKE3) {
            armSubController.setSensorLED(true);
            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE3.shoulderPosition);
            currentState = SAMPLE_INTAKE2_1;
        } else if (currentState == SAMPLE_INTAKE2_1) {
            if (transitionTimer.time() <= 200) return;
            transitionTimer.reset();

            armSubController.setClawPosition(ArmState.SAMPLE_INTAKE3.clawPosition);
            currentState = SAMPLE_INTAKE2_2;
        } else if (currentState == SAMPLE_INTAKE2_2) {
            if (transitionTimer.time() <= 250) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_INTAKE2.shoulderPosition);
            currentState = SAMPLE_INTAKE2_3;
        } else if (currentState == SAMPLE_INTAKE2_3) {
            if (transitionTimer.time() <= 450) return;
            transitionTimer.reset();

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor == YELLOW || sampleColor == allianceColor || isAutonomous) {
                armSubController.setSensorLED(false);
                armSubController.setTargetState(ArmState.SAMPLE_OUTTAKE1);

                if (scoringMode == CYCLING) {
                    slideSubController.setTargetPosition(SlideState.CYCLING1.targetPosition);
                    currentState = CYCLING0;
                    targetState = CYCLING1;
                } else if (scoringMode == SAMPLE) {
                    slideSubController.setTargetPosition(SlideState.SAMPLE_OUTTAKE.targetPosition);
                    currentState = SAMPLE_INTAKE3;
                    targetState = SAMPLE_OUTTAKE;
                }
            } else {
                armSubController.setClawPosition(ArmState.SAMPLE_INTAKE2.clawPosition);

                currentState = SAMPLE_INTAKE2;
                targetState = SAMPLE_INTAKE2;
            }
        } else if (currentState == SAMPLE_INTAKE_AUTO && targetState == SAMPLE_OUTTAKE_AUTO) {
            armSubController.setClawPosition(ArmState.SAMPLE_OUTTAKE1.clawPosition);
            currentState = SAMPLE_INTAKE_AUTO_1;
        } else if (currentState == SAMPLE_INTAKE_AUTO_1) {
            if (transitionTimer.time() <= 50) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_OUTTAKE_HIGH.targetPosition);
            currentState = SAMPLE_INTAKE_AUTO_2;
        } else if (currentState == SAMPLE_INTAKE_AUTO_2) {
            if (slideSubController.isCooking()) return;

            armSubController.setShoulderPosition(ArmState.SAMPLE_OUTTAKE2.shoulderPosition);
            armSubController.setElbowPosition(ArmState.SAMPLE_OUTTAKE2.elbowPosition);
            currentState = SAMPLE_OUTTAKE;
        }
    }

    private void completeSampleTransition() {
        completeSampleIntakeTransition();

        if (currentState == SAMPLE_INTAKE3) {
            if (slideSubController.isCooking()) return;

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor == YELLOW || sampleColor == allianceColor || isAutonomous) {
                armSubController.setShoulderPosition(ArmState.SAMPLE_OUTTAKE2.shoulderPosition);
                armSubController.setElbowPosition(ArmState.SAMPLE_OUTTAKE2.elbowPosition);
                pivotSubController.setTargetPosition(PivotState.SAMPLE_OUTTAKE.targetPosition);
                currentState = SAMPLE_INTAKE3_1;
            } else {
                armSubController.setTargetState(ArmState.SAMPLE_INTAKE2);
                slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE2.targetPosition);

                currentState = SAMPLE_INTAKE2;
                targetState = SAMPLE_INTAKE2;
            }
        } else if (currentState == SAMPLE_INTAKE3_1) {
             if (pivotSubController.isCooking()) return;

            armSubController.setWristPosition(ArmState.SAMPLE_OUTTAKE2.wristPosition);
            slideSubController.setTargetPosition(slideSubController.getScoringPosition());
            currentState = SAMPLE_OUTTAKE;
        } else if (currentState == SAMPLE_OUTTAKE && targetState == SAMPLE_INTAKE1) {
            armSubController.setClawPosition(ArmState.SAMPLE_OUTTAKE2.clawPosition);
            slideSubController.updateScoringPosition();
            currentState = SAMPLE_OUTTAKE_1;
        } else if (currentState == SAMPLE_OUTTAKE_1) {
            if (transitionTimer.time() <= 350) return;

            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            currentState = SAMPLE_OUTTAKE_2;
        } else if (currentState == SAMPLE_OUTTAKE_2) {
            if (transitionTimer.time() <= 500) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = SAMPLE_OUTTAKE_3;
        } else if (currentState == SAMPLE_OUTTAKE_3) {
            if (slideSubController.isCooking()) return;

            pivotSubController.setTargetPosition(PivotState.SAMPLE_INTAKE.targetPosition);
            currentState = SAMPLE_INTAKE1;
        } else if (currentState == SAMPLE_OUTTAKE && targetState == SAMPLE_INTAKE_AUTO) {
            armSubController.setClawPosition(ArmState.SAMPLE_OUTTAKE2.clawPosition);
            currentState = SAMPLE_OUTTAKE_AUTO_1;
        } else if (currentState == SAMPLE_OUTTAKE_AUTO_1) {
            if (transitionTimer.time() <= 350) return;

            armSubController.setTargetState(ArmState.SAMPLE_INTAKE1);
            currentState = SAMPLE_OUTTAKE_AUTO_2;
        } else if (currentState == SAMPLE_OUTTAKE_AUTO_2) {
            if (transitionTimer.time() <= 500) return;

            slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE1.targetPosition);
            currentState = SAMPLE_OUTTAKE_AUTO_3;
        } else if (currentState == SAMPLE_OUTTAKE_AUTO_3) {
            if (slideSubController.isCooking()) return;

            armSubController.setTargetState(ArmState.SAMPLE_INTAKE_AUTO);
            currentState = SAMPLE_INTAKE_AUTO;
        }
    }

    private void completeSpecimenTransition() {
        if (currentState == SPECIMEN_INTAKE && targetState == SPECIMEN_OUTTAKE) {
            armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE2.clawPosition);
            currentState = SPECIMEN_INTAKE_1;
        } else if (currentState == SPECIMEN_INTAKE_1) {
            if (transitionTimer.time() <= 50) return;

            armSubController.setSensorLED(true);
            slideSubController.setTargetPosition(SlideState.SPECIMEN_OUTTAKE.targetPosition);
            currentState = SPECIMEN_INTAKE_2;
        } else if (currentState == SPECIMEN_INTAKE_2) {
            if (slideSubController.isCooking()) return;
            transitionTimer.reset();

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor != NONE || isAutonomous) {
                armSubController.setSensorLED(false);
                armSubController.setShoulderPosition(ArmState.SPECIMEN_OUTTAKE.shoulderPosition);
                armSubController.setElbowPosition(ArmState.SPECIMEN_OUTTAKE.elbowPosition);
                currentState = SPECIMEN_INTAKE_3;
            } else {
                armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE1.clawPosition);
                slideSubController.setTargetPosition(SlideState.SPECIMEN_INTAKE.targetPosition);
                currentState = SPECIMEN_INTAKE;
                targetState = SPECIMEN_INTAKE;
            }
        } else if (currentState == SPECIMEN_INTAKE_3) {
            if (transitionTimer.time() <= 250) return;

            armSubController.setWristPosition(ArmState.SPECIMEN_OUTTAKE.wristPosition);
            currentState = SPECIMEN_OUTTAKE;
        } else if (currentState == SPECIMEN_OUTTAKE && targetState == SPECIMEN_INTAKE) {
            armSubController.setClawPosition(ArmState.SPECIMEN_INTAKE1.clawPosition);
            currentState = SPECIMEN_OUTTAKE_1;
        } else if (currentState == SPECIMEN_OUTTAKE_1) {
            if (transitionTimer.time() <= 750) return;

            armSubController.setTargetState(ArmState.SPECIMEN_INTAKE1);
            slideSubController.setTargetPosition(SlideState.SPECIMEN_INTAKE.targetPosition);
            currentState = SPECIMEN_INTAKE;
        }
    }

    private void completeCyclingTransition() {
        completeSampleIntakeTransition();

        if (currentState == CYCLING0) {
            if (slideSubController.isCooking()) return;

            SampleColor sampleColor = armSubController.getSampleColor();
            if (sampleColor == NONE) {
                armSubController.setTargetState(ArmState.SAMPLE_INTAKE2);
                slideSubController.setTargetPosition(SlideState.SAMPLE_INTAKE2.targetPosition);
                currentState = SAMPLE_INTAKE2;
                targetState = SAMPLE_INTAKE2;
            } else {
                currentState = CYCLING1;
                targetState = CYCLING1;
            }
        } else if (currentState == CYCLING1 && targetState == CYCLING2) {
            slideSubController.setTargetPosition(SlideState.CYCLING2.targetPosition);
            currentState = CYCLING2;
        } else if (currentState == CYCLING2 && targetState == SAMPLE_INTAKE1) {
            armSubController.setClawPosition(ArmState.SAMPLE_INTAKE1.clawPosition);
            currentState = CYCLING2_1;
        } else if (currentState == CYCLING2_1) {
            if (transitionTimer.time() <= 100) return;

            slideSubController.setTargetPosition(SlideState.CYCLING1.targetPosition);
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
            slideSubController.setTargetPosition(SlideState.ZERO.targetPosition);
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

        completeStateTransition();
        if (targetState.ordinal() <= 5) completeOpModeTransition();
        else if (scoringMode == SAMPLE) completeSampleTransition();
        else if (scoringMode == SPECIMEN) completeSpecimenTransition();
        else if (scoringMode == CYCLING) completeCyclingTransition();
        else if (scoringMode == ASCENT) completeAscentTransition();
    }

    public State getCurrentState() {
        return currentState;
    }

    public enum Alliance {RED, BLUE, NONE}

    public enum Direction {ANTICLOCKWISE, CLOCKWISE}

    public enum OpMode {AUTONOMOUS_SAMPLE, AUTONOMOUS_SPECIMEN, TELEOPUS}

    public enum ScoringMode {SAMPLE, CYCLING, SPECIMEN, ASCENT}

    public enum State {
        AUTONOMOUS_SAMPLE_START,
        AUTONOMOUS_SAMPLE_START_1,
        AUTONOMOUS_SPECIMEN_START,
        AUTONOMOUS_SPECIMEN_START_1,
        TELEOPUS_START,
        TELEOPUS_START_1,
        SAMPLE_INTAKE1,
        SAMPLE_INTAKE2,
        SAMPLE_INTAKE3,
        SAMPLE_OUTTAKE,
        SAMPLE_INTAKE_AUTO,
        SAMPLE_OUTTAKE_AUTO,
        SAMPLE_INTAKE2_1,
        SAMPLE_INTAKE2_2,
        SAMPLE_INTAKE2_3,
        SAMPLE_INTAKE2_4,
        SAMPLE_INTAKE3_1,
        SAMPLE_OUTTAKE_1,
        SAMPLE_OUTTAKE_2,
        SAMPLE_OUTTAKE_3,
        SAMPLE_INTAKE_AUTO_1,
        SAMPLE_INTAKE_AUTO_2,
        SAMPLE_OUTTAKE_AUTO_1,
        SAMPLE_OUTTAKE_AUTO_2,
        SAMPLE_OUTTAKE_AUTO_3,
        SPECIMEN_INTAKE,
        SPECIMEN_OUTTAKE,
        SPECIMEN_INTAKE_1,
        SPECIMEN_INTAKE_2,
        SPECIMEN_INTAKE_3,
        SPECIMEN_OUTTAKE_1,
        CYCLING0,
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
