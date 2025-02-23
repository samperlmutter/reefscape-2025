package frc.robot.subsystems.algae;

import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.sim.PhysicsSim;

public class AlgaeRollerIOReal implements AlgaeRollerIO {
    private final TalonFX roller;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> angularVel;

    public AlgaeRollerIOReal(boolean isSim) {
        roller = new TalonFX(AlgaeRollerConfig.ROLLER_ID, RIO_BUS);
        roller.getConfigurator().apply(AlgaeRollerConfig.TALON_FX_CONFIGURATION);

        appliedVolts = roller.getMotorVoltage();
        currentAmps = roller.getSupplyCurrent();
        position = roller.getPosition();
        angularVel = roller.getVelocity();

        if (isSim) {
            PhysicsSim.getInstance().addTalonFX(roller);
        }
    }

    @Override
    public void updateInputs(AlgaeRollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(appliedVolts, currentAmps, position, angularVel);

        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.positionRad = position.getValueAsDouble();
        inputs.velocityRadPerSec = angularVel.getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        roller.set(dutyCycle);
    }

    @Override
    public void stop() {
        roller.stopMotor();
    }
}
