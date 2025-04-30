package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.control.MotionMagicControl;
import frc.robot.util.control.State;
import frc.robot.util.control.StatefulSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends StatefulSubsystem<ElevatorPosition> implements MotionMagicControl {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        new Trigger(() -> inputs.isAtBottom)
                .debounce(2)
                .onTrue(runOnce(io::zeroPosition).andThen(runOnce(io::setNeutral)))
                .getAsBoolean();
        setDefaultCommand(runOnce(io::holdPosition));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public Angle currentPosition() {
        return inputs.primaryPositionRot;
    }

    @Override
    public Command moveTo(State<Angle> setpoint) {
        return run(() -> io.moveTo(setpoint.get())).until(this::hasReachedGoal);
    }

    @Override
    public boolean hasReachedGoal() {
        return currentPosition().isNear(getTargetState().get(), ElevatorConfig.HEIGHT_TOLERANCE);
    }
}
