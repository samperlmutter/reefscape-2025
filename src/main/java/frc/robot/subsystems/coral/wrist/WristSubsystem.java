package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.MotionMagicControl;
import frc.robot.util.control.State;
import frc.robot.util.control.StatefulSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends StatefulSubsystem<WristPositions> implements MotionMagicControl {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public WristSubsystem(WristIO io) {
        this.io = io;
        setDefaultCommand(run(io::holdPosition));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public Command moveTo(State<Angle> setpoint) {
        return runOnce(() -> io.moveTo(setpoint.get()));
    }

    @Override
    public Angle currentPosition() {
        return inputs.positionRot;
    }

    @Override
    public boolean hasReachedGoal() {
        return currentPosition().isNear(getTargetState().get(), WristConfigs.WRIST_TOLERANCE);
    }
}
