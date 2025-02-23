package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.util.state.StatefulSubsystem;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    public final ArmSubsystem arm;

    public final ElevatorSubsystem elevator;

    public final GrabberSubsystem grabber;

    public CoralManipulatorSystem(
            ArmSubsystem arm, ElevatorSubsystem elevator, GrabberSubsystem grabber) {
        super(CoralManipulatorState.IDLE);

        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        if (getCurrentState()
                        .getElevatorPosition()
                        .getHeight()
                        .lt(ElevatorPosition.SAFE_POSITION.getHeight())
                && getCurrentState() != targetState) {
            coralManipulatorCommand =
                    elevator.transitionTo(ElevatorPosition.SAFE_POSITION)
                            .andThen(arm.transitionTo(targetState.getArmPosition()))
                            .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                            .alongWith(grabber.transitionTo(targetState.getGrabberState()));
        } else {
            coralManipulatorCommand =
                    arm.transitionTo(targetState.getArmPosition())
                            .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                            .alongWith(grabber.transitionTo(targetState.getGrabberState()));
        }

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    @Override
    protected boolean isTransitionFinished() {
        return !arm.isTransitioning() && !elevator.isTransitioning() && !grabber.isTransitioning();
    }
}
