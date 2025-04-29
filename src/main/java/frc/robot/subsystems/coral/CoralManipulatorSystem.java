package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.subsystems.coral.wrist.WristPositions;
import frc.robot.subsystems.coral.wrist.WristSubsystem;
import frc.robot.util.state.StatefulSubsystem;
import java.util.Map;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    public final ArmSubsystem arm;

    public final ElevatorSubsystem elevator;

    public final WristSubsystem wrist;
    public final GrabberSubsystem grabber;

    private CoralManipulatorState queuedState = CoralManipulatorState.IDLE;

    public CoralManipulatorSystem(
            ArmSubsystem arm,
            ElevatorSubsystem elevator,
            GrabberSubsystem grabber,
            WristSubsystem wristSubsystem) {
        super(CoralManipulatorState.IDLE);

        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;
        this.wrist = wristSubsystem;
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

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        if (armIsTransitioningDangerZone()) {
            if (isElevatorGoingSafety()) {
                coralManipulatorCommand =
                        Commands.sequence(
                                elevator.transitionTo(ElevatorPosition.SAFE_POSITION),
                                arm.transitionTo(targetState.getArmPosition())
                                        .alongWith(
                                                elevator.transitionTo(
                                                        targetState.getElevatorPosition())),
                                grabber.transitionTo(targetState.getGrabberState()));
            } else {
                coralManipulatorCommand =
                        Commands.sequence(
                                elevator.transitionTo(ElevatorPosition.SAFE_POSITION),
                                arm.transitionTo(targetState.getArmPosition()),
                                elevator.transitionTo(targetState.getElevatorPosition()),
                                grabber.transitionTo(targetState.getGrabberState()));
            }
        } else {
            coralManipulatorCommand =
                    arm.transitionTo(targetState.getArmPosition())
                            .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                            .andThen(grabber.transitionTo(targetState.getGrabberState()));
        }

        if (isWristFirst(targetState)) {
            if (getCurrentState() == CoralManipulatorState.GROUND_INTAKE) {
                coralManipulatorCommand =
                        arm.transitionTo(ArmPosition.AWAY_BUMPER)
                                .andThen(wrist.transitionTo(targetState.getWristPosition()))
                                .andThen(coralManipulatorCommand);
            } else {
                coralManipulatorCommand =
                        wrist.transitionTo(targetState.getWristPosition())
                                .andThen(coralManipulatorCommand);
            }
        } else {
            coralManipulatorCommand =
                    coralManipulatorCommand.andThen(
                            wrist.transitionTo(targetState.getWristPosition()));
        }

        coralManipulatorCommand =
                wrist.transitionTo(getCurrentState().getWristPosition())
                        .andThen(coralManipulatorCommand);

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    private boolean isElevatorGoingSafety() {
        return wantedState
                .getElevatorPosition()
                .getHeight()
                .gt(ElevatorPosition.SAFE_POSITION.getHeight());
    }

    boolean armIsTransitioningDangerZone() {
        return arm.currentPosition().lt(ArmPosition.POS_L1.getAngle())
                || wantedState.getArmPosition().getAngle().lt(ArmPosition.POS_L1.getAngle());
    }

    public boolean isWristFirst(CoralManipulatorState targetState) {
        return getCurrentState().getWristPosition() == WristPositions.UNSAFE
                || targetState.getWristPosition() == WristPositions.SAFE;
    }

    @Override
    protected boolean isTransitionFinished() {
        return !arm.isTransitioning() && !elevator.isTransitioning() && !grabber.isTransitioning();
    }
}
