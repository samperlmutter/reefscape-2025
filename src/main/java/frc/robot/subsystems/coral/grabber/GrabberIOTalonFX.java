package frc.robot.subsystems.coral.grabber;

import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class GrabberIOTalonFX implements GrabberIO {
    private final TalonFX claw;
    private final Canandcolor clawSwitch;

    public GrabberIOTalonFX() {
        clawSwitch = new Canandcolor(GrabberConfig.GRABBER_CANANDCOLOR);
        claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(claw);
        }
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        double prox = clawSwitch.getProximity();
        boolean hasCoral = inputs.hasCoral;

        if (prox < 0.2) {
            hasCoral = true;
        } else if (prox > 0.32) {
            hasCoral = false;
        }

        inputs.updateAll(
                claw.getPosition().getValue(),
                claw.getVelocity().getValue(),
                claw.getMotorVoltage().getValue(),
                claw.getStatorCurrent().getValue(),
                hasCoral);
    }

    @Override
    public void rollIn() {
        claw.set(GrabberConfig.INTAKE);
    }

    @Override
    public void rollOut() {
        claw.set(GrabberConfig.OUTSPIT);
    }

    @Override
    public void stop() {
        claw.stopMotor();
    }
}
