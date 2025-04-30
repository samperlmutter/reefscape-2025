package frc.robot.subsystems.coral.elevator;

import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID, RIO_BUS);
    private final TalonFX secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID, RIO_BUS);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);

    private final MotionMagicTorqueCurrentFOC magicRequest;

    private final StatusSignal<Angle> primaryPositionRot = primaryElevatorMotor.getPosition();
    private final StatusSignal<Angle> secondaryPositionRot = secondaryElevatorMotor.getPosition();
    private final StatusSignal<AngularVelocity> primaryVelocityRotPerSec = primaryElevatorMotor.getVelocity();
    private final StatusSignal<AngularVelocity> secondaryVelocityRotPerSec = secondaryElevatorMotor.getVelocity();
    private final StatusSignal<Voltage> primaryAppliedVolts = primaryElevatorMotor.getMotorVoltage();
    private final StatusSignal<Voltage> secondaryAppliedVolts = secondaryElevatorMotor.getMotorVoltage();
    private final StatusSignal<Current> primaryCurrentAmps = primaryElevatorMotor.getStatorCurrent();
    private final StatusSignal<Current> secondaryCurrentAmps = secondaryElevatorMotor.getSupplyCurrent();

    public ElevatorIOTalonFX() {
        primaryElevatorMotor.getConfigurator().apply(ElevatorConfig.primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(ElevatorConfig.secondaryTalonFXConfigs);
        magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(Robot.isReal() ? 0 : 1);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
        }
    }

    private boolean isAtBottom() {
        return !magSwitch.get();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return primaryElevatorMotor.setControl(magicRequest.withPosition(setpoint));
    }

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                primaryPositionRot,
                secondaryPositionRot,
                primaryVelocityRotPerSec,
                secondaryVelocityRotPerSec,
                primaryAppliedVolts,
                secondaryAppliedVolts,
                primaryCurrentAmps,
                secondaryCurrentAmps);
        inputs.updateAll(
                primaryPositionRot.getValue(),
                secondaryPositionRot.getValue(),
                primaryVelocityRotPerSec.getValue(),
                secondaryVelocityRotPerSec.getValue(),
                primaryAppliedVolts.getValue(),
                secondaryAppliedVolts.getValue(),
                primaryCurrentAmps.getValue(),
                secondaryCurrentAmps.getValue(),
                isAtBottom());
    }

    @Override
    public void setNeutral() {
        primaryElevatorMotor.setControl(new NeutralOut());
    }

    @Override
    public void zeroPosition() {
        primaryElevatorMotor.setPosition(0);
        secondaryElevatorMotor.setPosition(0);
    }

    @Override
    public StatusCode holdPosition() {
        return primaryElevatorMotor.setControl(magicRequest.withPosition(primaryPositionRot.getValue()));
    }
}
