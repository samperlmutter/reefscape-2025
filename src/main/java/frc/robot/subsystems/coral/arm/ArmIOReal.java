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
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOReal implements ArmIO {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
    private final MotionMagicTorqueCurrentFOC magicRequest;

    StatusSignal<Voltage> appliedVolts = armKraken.getMotorVoltage();
    StatusSignal<Current> currentAmps = armKraken.getSupplyCurrent();
    StatusSignal<Angle> position = armKraken.getPosition();
    StatusSignal<AngularVelocity> angularVel = armKraken.getVelocity();

    @AutoLogOutput(key = "Arm/Target Positiion")
    StatusSignal<Double> targetPositionRot = armKraken.getClosedLoopReference();

    @AutoLogOutput(key = "Arm/Target Velocity")
    StatusSignal<Double> targetVelocityRotPerSec = armKraken.getClosedLoopReferenceSlope();

    @AutoLogOutput(key = "Arm/Target Error")
    StatusSignal<Double> targetError = armKraken.getClosedLoopError();

    @AutoLogOutput(key = "Arm/Closed Loop Output")
    StatusSignal<Double> closedLoopOutput = armKraken.getClosedLoopOutput();

    @AutoLogOutput(key = "Closed Loop P Output")
    StatusSignal<Double> closedLoopPOutput = armKraken.getClosedLoopProportionalOutput();

    @AutoLogOutput(key = "Closed Loop I Output")
    StatusSignal<Double> closedLoopIOutput = armKraken.getClosedLoopIntegratedOutput();

    @AutoLogOutput(key = "Closed Loop D Output")
    StatusSignal<Double> closedLoopDOutput = armKraken.getClosedLoopDerivativeOutput();

    @AutoLogOutput(key = "Arm/Active Slot")
    StatusSignal<Integer> activeSlot = armKraken.getClosedLoopSlot();

    @AutoLogOutput(key = "Arm/Duty Cycle")
    StatusSignal<Double> dutyCycle = armKraken.getDutyCycle();

    public ArmIOReal(boolean isSim) {
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.RIO_BUS);
        magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(isSim ? 0 : 1);

        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        if (isSim) {
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
}
