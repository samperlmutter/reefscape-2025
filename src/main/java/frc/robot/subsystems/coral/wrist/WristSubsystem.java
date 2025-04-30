package frc.robot.subsystems.coral.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.State;
import frc.robot.util.sim.PhysicsSim;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor = new TalonFX(WristConfigs.WRIST_KRAKEN_ID, Constants.RIO_BUS);
    private final CANcoder wristEncoder = new CANcoder(WristConfigs.WRIST_CANCODER_ID);
    private final StatusSignal<Angle> wristPosition = wristMotor.getPosition();
    private final StatusSignal<Double> targetWristPosition = wristMotor.getClosedLoopReference();
    private final PositionVoltage motionRequest = new PositionVoltage(0).withSlot(0);

    public WristSubsystem() {
        wristMotor.getConfigurator().apply(WristConfigs.wristMotorConfigs);
        wristEncoder.getConfigurator().apply(WristConfigs.wristEncoderConfigs);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(wristMotor, wristEncoder);
        }
        wristMotor.setPosition(WristPositions.SAFE.get());
    }

    public Command moveWristHorizontal() {
        return runOnce(() -> moveTo(WristPositions.SAFE));
    }

    public Command moveWristVertical() {
        return runOnce(() -> moveTo(WristPositions.UNSAFE));
    }

    public StatusSignal<Angle> currentStateSignal() {
        return wristPosition;
    }

    public Angle determineSetpoint(WristPositions targetState) {
        return targetState == WristPositions.HOLD ? wristPosition.getValue() : targetState.get();
    }

    public Command moveTo(State<Angle> setpoint) {
        return runOnce(() -> wristMotor.setControl(motionRequest.withPosition(setpoint.get())));
    }

    public Angle getCurrentPosition() {
        return wristPosition.getValue();
    }

    public Angle getTargetPosition() {
        return Units.Rotations.of(targetWristPosition.getValue());
    }
}
