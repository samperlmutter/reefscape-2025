package frc.robot.util.control;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface MotionMagicControl {
    Command moveTo(State<Angle> setpoint);

    Angle currentPosition();

    boolean hasReachedGoal();
}
