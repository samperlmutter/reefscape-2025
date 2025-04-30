package frc.robot.util.control;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatefulSubsystem<S extends State<?>> extends SubsystemBase {
    private S currentState, targetState;

    public StatefulSubsystem() {}

    public S getCurrentState() {
        return currentState;
    }

    public void setCurrentState(S currentState) {
        this.currentState = currentState;
    }

    public S getTargetState() {
        return targetState;
    }

    public void setTargetState(S targetState) {
        this.targetState = targetState;
    }
}
