package frc.robot.util.control;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public interface MotionMagicControl<S extends State<?>> {
    default StatusCode moveTo(Angle setpoint) {
        return StatusCode.OK;
    }

    default Angle currentPosition() {
        return Units.Rotations.zero();
    }
}
