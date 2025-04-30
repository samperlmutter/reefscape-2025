package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotionMagicControl;
import frc.robot.util.State;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements MotionMagicControl {
    ArmIO io;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
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
        return false;
    }
}
