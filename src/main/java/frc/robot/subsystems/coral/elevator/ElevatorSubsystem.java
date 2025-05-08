package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.control.StatefulSetpointSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends StatefulSetpointSubsystem<ElevatorPosition, ElevatorIO> {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        super(io, ElevatorConfig.HEIGHT_TOLERANCE);
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

    public Angle currentPosition() {
        return inputs.primaryPositionRot;
    }
}
