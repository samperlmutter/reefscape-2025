package frc.robot.subsystems.coral.arm;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.control.StatefulSetpointSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends StatefulSetpointSubsystem<ArmPosition, ArmIO> {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        super(io, ArmConfig.ANGLE_TOLERANCE);
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public Angle currentPosition() {
        return inputs.positionRot;
    }
}
