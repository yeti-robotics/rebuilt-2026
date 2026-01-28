package frc.robot.subsystems.linslide;

public enum LinSlidePosition {
    STOW(0.0),
    DEPLOY(3.29);

    private final double position;

    LinSlidePosition(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
