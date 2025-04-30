package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.control.State;

public enum WristPositions implements State<Angle> {
    SAFE(0),
    UNSAFE(0.25),
    HOLD(-1);

    private final Angle angle;

    WristPositions(double angle) {
        this.angle = Units.Rotations.of(angle);
    }

    @Override
    public Angle get() {
        return angle;
    }
}
