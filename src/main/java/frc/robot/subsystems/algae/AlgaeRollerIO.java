package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollerIO {
    @AutoLog
    public static class AlgaeRollerIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(AlgaeRollerIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void stop() {}
}
