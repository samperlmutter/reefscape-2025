package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public Angle positionRot = Units.Rotations.of(0.0);
        public AngularVelocity velocityRotPerSec = Units.RotationsPerSecond.of(0.0);
        public Voltage appliedVolts = Units.Volts.of(.0);
        public Current currentAmps = Units.Amps.of(0.0);
        public boolean hasCoral = false;

        public void updateAll(
                Angle positionRot,
                AngularVelocity velocityRotPerSec,
                Voltage appliedVolts,
                Current current,
                boolean hasCoral) {
            this.positionRot = positionRot;
            this.velocityRotPerSec = velocityRotPerSec;
            this.appliedVolts = appliedVolts;
            this.currentAmps = current;
            this.hasCoral = hasCoral;
        }
    }

    default void updateInputs(GrabberIOInputs inputs) {}

    default void rollIn() {}

    default void rollOut() {}

    default void stop() {}
}
