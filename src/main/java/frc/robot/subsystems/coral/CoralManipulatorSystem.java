package frc.robot.subsystems.coral;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.subsystems.coral.wrist.WristPositions;
import frc.robot.subsystems.coral.wrist.WristSubsystem;
import frc.robot.util.control.StatefulSubsystem;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final GrabberSubsystem grabber;

    private CoralManipulatorState queuedState = CoralManipulatorState.IDLE;

    public CoralManipulatorSystem(
            ElevatorSubsystem elevator, ArmSubsystem arm, GrabberSubsystem grabber, WristSubsystem wristSubsystem) {
        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;
        this.wrist = wristSubsystem;

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

    public Command moveToCmd(CoralManipulatorState targetState) {
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
            if (getCurrentState().isPresent() && getCurrentState().get() == CoralManipulatorState.GROUND_INTAKE) {
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

        return expose(coralManipulatorCommand);
    }

    @Override
    public Command moveTo(CoralManipulatorState setpoint) {
        return Commands.sequence(
                Commands.sequence(
                        runOnce(() -> setTargetState(Optional.of(setpoint))),
                        runOnce(() -> setCurrentState(Optional.empty()))),
                moveToCmd(setpoint).until(this::isDoneMoving),
                Commands.sequence(
                        runOnce(() -> setCurrentState(Optional.of(setpoint))),
                        runOnce(() -> setTargetState(Optional.empty()))));
    }

    @Override
    public void setTargetState(Optional<CoralManipulatorState> targetState) {
        super.setTargetState(targetState);
        elevator.setTargetState(Optional.of(targetState.get().getElevatorPosition()));
        arm.setTargetState(Optional.of(targetState.get().getArmPosition()));
        wrist.setTargetState(Optional.of(targetState.get().getWristPosition()));
        grabber.setTargetState(Optional.of(targetState.get().getGrabberState()));
    }

    private boolean isDoneMoving() {
        return elevator.hasReachedGoal()
                && arm.hasReachedGoal()
                && wrist.hasReachedGoal()
                && grabber.getCurrentCommand() == null;
    }

    public Command releaseCoral() {
        return expose(grabber.moveTo(GrabberState.ROLL_OUT));
    }

    private boolean isElevatorGoingSafety() {
        return getTargetState().isPresent()
                && getTargetState().get().getElevatorPosition().get().gt(ElevatorPosition.SAFE_POSITION.get());
    }

    boolean armIsTransitioningDangerZone() {
        return arm.currentPosition().lt(ArmPosition.POS_L1.get())
                || (getTargetState().isPresent()
                        && getTargetState().get().getArmPosition().get().lt(ArmPosition.POS_L1.get()));
    }

    public boolean isWristFirst(CoralManipulatorState targetState) {
        return getCurrentState().isPresent() && getCurrentState().get().getWristPosition() == WristPositions.UNSAFE
                || targetState.getWristPosition() == WristPositions.SAFE;
    }

    public Angle getArmPosition() {
        return arm.currentPosition();
    }

    public Angle getElevatorPosition() {
        return elevator.currentPosition();
    }

    public Angle getWristPosition() {
        return wrist.currentPosition();
    }
}
