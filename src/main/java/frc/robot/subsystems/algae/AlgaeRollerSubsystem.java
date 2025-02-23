package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollerSubsystem extends SubsystemBase {
    private final AlgaeRollerIO io;
    private final AlgaeRollerIOInputsAutoLogged inputs = new AlgaeRollerIOInputsAutoLogged();

    public AlgaeRollerSubsystem(AlgaeRollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeRoller", inputs);
    }

    private void setRollerSpeed(double speed) {
        io.setDutyCycle(speed);
    }

    private void stop() {
        io.stop();
    }

    public Command spinRoller(double speed) {
        return startEnd(() -> setRollerSpeed(speed), this::stop);
    }
}
