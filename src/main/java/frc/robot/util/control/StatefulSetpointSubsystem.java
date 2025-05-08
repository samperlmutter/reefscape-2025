package frc.robot.util.control;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;

public class StatefulSetpointSubsystem<S extends State<Angle>, I extends MotionMagicControl<S>>
        extends StatefulSubsystem<S> {
    protected final double tolerance;
    protected final I io;

    public StatefulSetpointSubsystem(I io, double tolerance) {
        this.io = io;
        this.tolerance = tolerance;
    }

    @Override
    public Command moveTo(S setpoint) {
        return Commands.sequence(
                Commands.sequence(
                        runOnce(() -> setTargetState(Optional.of(setpoint))),
                        runOnce(() -> setCurrentState(Optional.empty()))),
                run(() -> io.moveTo(setpoint.get())).until(this::hasReachedGoal),
                Commands.sequence(
                        runOnce(() -> setCurrentState(Optional.of(setpoint))),
                        runOnce(() -> setTargetState(Optional.empty()))));
    }

    public boolean hasReachedGoal() {
        if (getTargetState().isPresent()) {
            return io.currentPosition().isNear(getTargetState().get().get(), tolerance);
        }
        return false;
    }

    public void holdPosition() {
        io.moveTo(io.currentPosition());
    }
}
