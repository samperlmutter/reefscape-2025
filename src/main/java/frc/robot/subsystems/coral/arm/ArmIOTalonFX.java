package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final CANcoder armEncoder;
    private final MotionMagicTorqueCurrentFOC magicRequest;

    public ArmIOTalonFX() {
        armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.RIO_BUS);
        magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(Robot.isReal() ? 0 : 2);

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
        inputs.updateAll(
                armKraken.getPosition().getValue(),
                armKraken.getVelocity().getValue(),
                armKraken.getMotorVoltage().getValue(),
                armKraken.getStatorCurrent().getValue());
    }
}
