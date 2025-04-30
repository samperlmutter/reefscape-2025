package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MotionMagicControl;
import frc.robot.util.State;

import static frc.robot.subsystems.coral.grabber.GrabberState.OFF;
import static frc.robot.subsystems.coral.grabber.GrabberState.ROLL_OUT;

public class GrabberSubsystem extends SubsystemBase implements MotionMagicControl {
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
    public Command moveTo(State setpoint) {
        return switch ((GrabberState) setpoint) {
            case OFF -> runOnce(io::stop);
            case ROLL_IN -> runOnce(io::rollIn);
            case ROLL_OUT -> runOnce(io::rollOut);
        };
    }

    @Override
    public Angle currentPosition() {
        return null;
    }

    @Override
    public boolean hasReachedGoal() {
        return false;
    }
}
