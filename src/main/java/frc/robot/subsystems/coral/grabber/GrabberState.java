package frc.robot.subsystems.coral.grabber;

import frc.robot.util.State;

public enum GrabberState implements State<Double> {
    ROLL_OUT(GrabberConfig.OUTSPIT),
    ROLL_IN(GrabberConfig.INTAKE),
    OFF(0);

    private final double speed;

    GrabberState(double speed) {
        this.speed = speed;
    }

    @Override
    public Double get() {
        return speed;
    }
}
