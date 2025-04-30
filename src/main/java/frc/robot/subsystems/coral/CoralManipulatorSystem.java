package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.subsystems.coral.wrist.WristPositions;
import frc.robot.subsystems.coral.wrist.WristSubsystem;
import java.util.Map;
import java.util.Set;

public class CoralManipulatorSystem extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final GrabberSubsystem grabber;

    private CoralManipulatorState queuedState = CoralManipulatorState.IDLE;
    private CoralManipulatorState currentState, wantedState;

    public CoralManipulatorSystem(
            ElevatorSubsystem elevator, ArmSubsystem arm, GrabberSubsystem grabber, WristSubsystem wristSubsystem) {
        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;
        this.wrist = wristSubsystem;
        currentState = CoralManipulatorState.IDLE;
        wantedState = CoralManipulatorState.IDLE;

        grabber.hasCoralTrigger.onTrue(transitionTo(CoralManipulatorState.STOWED));
    }

    private Command expose(Command internal) {
        var internalProxy = internal.asProxy();
        internalProxy.addRequirements(this);
        return internalProxy;
    }

    private CoralManipulatorState getQueuedState() {
        return queuedState;
    }

    public void queueState(CoralManipulatorState state) {
        queuedState = state;
    }

    public Command selectQueuedStateCommand() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1, transitionTo(CoralManipulatorState.L1),
                        CoralManipulatorState.L2, transitionTo(CoralManipulatorState.L2),
                        CoralManipulatorState.L3, transitionTo(CoralManipulatorState.L3),
                        CoralManipulatorState.L4, transitionTo(CoralManipulatorState.L4)),
                this::getQueuedState);
    }

    public Command scoreState() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1, transitionTo(CoralManipulatorState.SCORE_L1),
                        CoralManipulatorState.L2, transitionTo(CoralManipulatorState.SCORE_L2),
                        CoralManipulatorState.L3, transitionTo(CoralManipulatorState.SCORE_L3),
                        CoralManipulatorState.L4, transitionTo(CoralManipulatorState.SCORE_L4)),
                this::getQueuedState);
    }

    public Command setQueueState(CoralManipulatorState queuedState) {
        return runOnce(() -> queueState(queuedState));
    }

    public Command transitionTo(CoralManipulatorState targetState) {
        return Commands.defer(() -> moveTo(targetState), Set.of(this));
    }

    private Command moveTo(CoralManipulatorState targetState) {
        wantedState = targetState;
        Command coralManipulatorCommand;

        if (armIsTransitioningDangerZone()) {
            if (isElevatorGoingSafety()) {
                coralManipulatorCommand = Commands.sequence(
                        elevator.moveTo(ElevatorPosition.SAFE_POSITION),
                        arm.moveTo(targetState.getArmPosition())
                                .alongWith(elevator.moveTo(targetState.getElevatorPosition())),
                        grabber.moveTo(targetState.getGrabberState()));
            } else {
                coralManipulatorCommand = Commands.sequence(
                        elevator.moveTo(ElevatorPosition.SAFE_POSITION),
                        arm.moveTo(targetState.getArmPosition()),
                        elevator.moveTo(targetState.getElevatorPosition()),
                        grabber.moveTo(targetState.getGrabberState()));
            }
        } else {
            coralManipulatorCommand = arm.moveTo(targetState.getArmPosition())
                    .alongWith(elevator.moveTo(targetState.getElevatorPosition()))
                    .andThen(grabber.moveTo(targetState.getGrabberState()));
        }

        if (isWristFirst(targetState)) {
            if (currentState == CoralManipulatorState.GROUND_INTAKE) {
                coralManipulatorCommand = arm.moveTo(ArmPosition.AWAY_BUMPER)
                        .andThen(wrist.moveTo(targetState.getWristPosition()))
                        .andThen(coralManipulatorCommand);
            } else {
                coralManipulatorCommand =
                        wrist.moveTo(targetState.getWristPosition()).andThen(coralManipulatorCommand);
            }
        } else {
            coralManipulatorCommand = coralManipulatorCommand.andThen(wrist.moveTo(targetState.getWristPosition()));
        }

        return expose(coralManipulatorCommand.finallyDo(() -> currentState = wantedState));
    }

    public Command releaseCoral() {
        return expose(grabber.moveTo(GrabberState.ROLL_OUT));
    }

    private boolean isElevatorGoingSafety() {
        return wantedState.getElevatorPosition().get().gt(ElevatorPosition.SAFE_POSITION.get());
    }

    boolean armIsTransitioningDangerZone() {
        return arm.currentPosition().lt(ArmPosition.POS_L1.get())
                || wantedState.getArmPosition().get().lt(ArmPosition.POS_L1.get());
    }

    public boolean isWristFirst(CoralManipulatorState targetState) {
        return currentState.getWristPosition() == WristPositions.UNSAFE
                || targetState.getWristPosition() == WristPositions.SAFE;
    }
}
