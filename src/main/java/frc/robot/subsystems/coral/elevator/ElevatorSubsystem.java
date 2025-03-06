package frc.robot.subsystems.coral.elevator;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sim.SimulatableMechanism;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem
        extends StatefulSetpointSubsystem<ElevatorPosition, AngleUnit, Angle, MutAngle>
        implements SimulatableMechanism {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        super(
                ElevatorPosition.BOTTOM,
                StateUtils.mutableRotationSetpoint(),
                Units.Rotations.of(ElevatorConfig.HEIGHT_TOLERANCE));
        this.io = io;

        new Trigger(() -> inputs.isAtBottom)
                .debounce(2)
                .onTrue(
                        runOnce(io::zeroPosition)
                                .andThen(
                                        transitionTo(ElevatorPosition.BOTTOM)
                                                .andThen(runOnce(io::setNeutral))))
                .getAsBoolean();
    }

    @Override
    protected void runPeriodic() {
        super.runPeriodic();
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        Logger.recordOutput("Elevator/Current State", getCurrentState());
        Logger.recordOutput("Elevator/Wanted State", transitioningTo().orElse(null));
        Logger.recordOutput("Elevator/Is Transitioning?", isTransitioning());
        Logger.recordOutput("Elevator/Current State", getCurrentState());
        Logger.recordOutput("Elevator/Default State", getDefaultState());
    }

    @Override
    public Angle currentPosition() {
        return inputs.primaryPositionRot;
    }

    @Override
    public Angle determineSetpoint(ElevatorPosition targetState) {
        return targetState == ElevatorPosition.HOLD
                ? inputs.primaryPositionRot
                : targetState.getHeight();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return io.moveTo(setpoint);
    }

    @Override
    public double updateMechPos() {
        return inchesToMeters(inputs.primaryPositionRot.magnitude());
    }
}
