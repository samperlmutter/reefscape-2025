package frc.robot.util.control;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class StatefulSubsystem<S extends State<?>> extends SubsystemBase {
    private Optional<S> currentState, targetState;

    public StatefulSubsystem() {}

    public Optional<S> getCurrentState() {
        return currentState;
    }

    public void setCurrentState(Optional<S> currentState) {
        this.currentState = currentState;
    }

    public Optional<S> getTargetState() {
        return targetState;
    }

    public void setTargetState(Optional<S> targetState) {
        this.targetState = targetState;
    }
}
