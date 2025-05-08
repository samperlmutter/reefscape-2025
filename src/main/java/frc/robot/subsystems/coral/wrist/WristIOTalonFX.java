package frc.robot.subsystems.coral.wrist;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class WristIOTalonFX implements WristIO {
    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;
    private final PositionVoltage positionVoltage;

    public WristIOTalonFX() {
        wristMotor = new TalonFX(WristConfigs.WRIST_KRAKEN_ID, Constants.RIO_BUS);
        wristEncoder = new CANcoder(WristConfigs.WRIST_CANCODER_ID);
        positionVoltage = new PositionVoltage(0).withSlot(Robot.isReal() ? 0 : 1);

        wristMotor.getConfigurator().apply(WristConfigs.wristMotorConfigs);
        wristEncoder.getConfigurator().apply(WristConfigs.wristEncoderConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(wristMotor, wristEncoder);
        }
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return wristMotor.setControl(positionVoltage.withPosition(setpoint));
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.updateAll(
                wristMotor.getPosition().getValue(),
                wristMotor.getVelocity().getValue(),
                wristMotor.getMotorVoltage().getValue(),
                wristMotor.getStatorCurrent().getValue());
    }
}
