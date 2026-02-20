package frc.robot.subsystems.hood;

public enum HoodPositions {
    STOW(0.0),
    BUMP(0.0),
    DEPLOY(1.0);

    private final double position;

    HoodPositions(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
