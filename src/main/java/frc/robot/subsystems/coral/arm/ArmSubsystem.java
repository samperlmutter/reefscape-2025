package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.util.sim.SimulatableMechanism;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends StatefulSetpointSubsystem<ArmPosition, AngleUnit, Angle, MutAngle>
        implements SimulatableMechanism {
    ArmIO io;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        super(
                ArmPosition.HOLD,
                StateUtils.mutableRotationSetpoint(),
                Units.Rotations.of(ArmConfig.ANGLE_TOLERANCE));
        this.io = io;
    }

    @Override
    public void runPeriodic() {
        super.runPeriodic();
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    @Override
    public Angle determineSetpoint(ArmPosition targetState) {
        return targetState == ArmPosition.HOLD ? inputs.positionRot : targetState.getAngle();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return io.moveTo(setpoint);
    }

    @Override
    public Angle currentPosition() {
        return inputs.positionRot;
    }

    @Override
    public double updateMechPos() {
        return (inputs.positionRot.magnitude() * 360.0) - 90;
    }
}
