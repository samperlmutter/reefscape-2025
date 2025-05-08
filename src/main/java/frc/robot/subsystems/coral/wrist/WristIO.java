package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.control.MotionMagicControl;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO extends MotionMagicControl<WristPositions> {
    @AutoLog
    public static class WristIOInputs {
        public Angle positionRot = Units.Rotations.of(0.0);
        public AngularVelocity velocityRotPerSec = Units.RotationsPerSecond.of(0.0);
        public Voltage appliedVolts = Units.Volts.of(.0);
        public Current currentAmps = Units.Amps.of(0.0);

        public void updateAll(
                Angle positionRot, AngularVelocity velocityRotPerSec, Voltage appliedVolts, Current current) {
            this.positionRot = positionRot;
            this.velocityRotPerSec = velocityRotPerSec;
            this.appliedVolts = appliedVolts;
            this.currentAmps = current;
        }
    }

    default void updateInputs(WristIOInputs inputs) {}
}
