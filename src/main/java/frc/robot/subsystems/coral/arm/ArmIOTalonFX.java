package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicTorqueCurrentFOC magicRequest;

    StatusSignal<Voltage> appliedVolts = armKraken.getMotorVoltage();
    StatusSignal<Current> currentAmps = armKraken.getSupplyCurrent();
    StatusSignal<Angle> position = armKraken.getPosition();
    StatusSignal<AngularVelocity> angularVel = armKraken.getVelocity();

    public ArmIOTalonFX() {
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.RIO_BUS);
        magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(Robot.isReal() ? 0 : 1);

        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armKraken, armEncoder);
        }
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return armKraken.setControl(magicRequest.withPosition(setpoint));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, angularVel, appliedVolts, currentAmps);
        inputs.updateAll(position.getValue(), angularVel.getValue(), appliedVolts.getValue(), currentAmps.getValue());
    }

    @Override
    public StatusCode holdPosition() {
        return armKraken.setControl(magicRequest.withPosition(position.getValue()));
    }
}
