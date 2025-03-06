package frc.robot.subsystems.coral.elevator;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public Angle primaryPositionRot = Units.Rotations.of(0);
        public Angle secondaryPositionRot = Units.Rotations.of(0);
        public AngularVelocity primaryVelocityRotPerSec = Units.RotationsPerSecond.of(0);
        public AngularVelocity secondaryVelocityRotPerSec = Units.RotationsPerSecond.of(0);
        public Voltage primaryAppliedVolts = Units.Volts.of(0);
        public Voltage secondaryAppliedVolts = Units.Volts.of(0);
        public Current primaryCurrentAmps = Units.Amps.of(0);
        public Current secondaryCurrentAmps = Units.Amps.of(0);
        public boolean isAtBottom = false;

        public void updateAll(
                Angle primaryPositionRot,
                Angle secondaryPositionRot,
                AngularVelocity primaryVelocityRotPerSec,
                AngularVelocity secondaryVelocityRotPerSec,
                Voltage primaryAppliedVolts,
                Voltage secondaryAppliedVolts,
                Current primaryCurrent,
                Current secondaryCurrent,
                boolean isAtBottom) {
            this.primaryPositionRot = primaryPositionRot;
            this.secondaryPositionRot = secondaryPositionRot;
            this.primaryVelocityRotPerSec = primaryVelocityRotPerSec;
            this.secondaryVelocityRotPerSec = secondaryVelocityRotPerSec;
            this.primaryAppliedVolts = primaryAppliedVolts;
            this.secondaryAppliedVolts = secondaryAppliedVolts;
            this.primaryCurrentAmps = primaryCurrent;
            this.secondaryCurrentAmps = secondaryCurrent;
            this.isAtBottom = isAtBottom;
        }
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default StatusCode moveTo(Angle setpoint) {
        return StatusCode.OK;
    }

    default void setNeutral() {}

    default void zeroPosition() {}
}
