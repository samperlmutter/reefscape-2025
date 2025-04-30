package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.MotionMagicControl;
import frc.robot.util.control.State;
import frc.robot.util.control.StatefulSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends StatefulSubsystem<ArmPosition> implements MotionMagicControl {
    ArmIO io;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        setDefaultCommand(runOnce(io::holdPosition));
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
        return currentPosition().isNear(getTargetState().get(), ArmConfig.ANGLE_TOLERANCE);
    }
}
