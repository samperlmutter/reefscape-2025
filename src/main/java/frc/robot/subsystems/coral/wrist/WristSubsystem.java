package frc.robot.subsystems.coral.wrist;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.control.StatefulSetpointSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends StatefulSetpointSubsystem<WristPositions, WristIO> {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public WristSubsystem(WristIO io) {
        super(io, WristConfigs.WRIST_TOLERANCE);
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public Angle currentPosition() {
        return inputs.positionRot;
    }
}
