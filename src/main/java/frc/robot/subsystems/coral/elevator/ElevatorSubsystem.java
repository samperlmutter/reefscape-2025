package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MotionMagicControl;
import frc.robot.util.State;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase implements MotionMagicControl {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        new Trigger(() -> inputs.isAtBottom)
                .debounce(2)
                .onTrue(runOnce(io::zeroPosition).andThen(runOnce(io::setNeutral)))
                .getAsBoolean();
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
        return runOnce(() -> io.moveTo(setpoint.get()));
    }

    @Override
    public boolean hasReachedGoal() {
        return false;
    }
}
