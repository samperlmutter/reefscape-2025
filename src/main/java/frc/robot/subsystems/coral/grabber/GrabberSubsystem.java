package frc.robot.subsystems.coral.grabber;

import static frc.robot.subsystems.coral.grabber.GrabberState.OFF;
import static frc.robot.subsystems.coral.grabber.GrabberState.ROLL_OUT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.control.StatefulSubsystem;
import org.littletonrobotics.junction.Logger;

public class GrabberSubsystem extends StatefulSubsystem<GrabberState> {
    public final Trigger hasCoralTrigger;
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();
    private final GrabberState currentState;

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
        currentState = OFF;
        hasCoralTrigger = new Trigger(() -> inputs.hasCoral).onTrue(moveTo(OFF));
        new Trigger(() -> currentState == ROLL_OUT).debounce(1).onTrue(moveTo(OFF));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    @Override
    public Command moveTo(GrabberState setpoint) {
        return switch (setpoint) {
            case OFF -> runOnce(io::stop);
            case ROLL_IN -> runOnce(io::rollIn);
            case ROLL_OUT -> runOnce(io::rollOut);
        };
    }
}
