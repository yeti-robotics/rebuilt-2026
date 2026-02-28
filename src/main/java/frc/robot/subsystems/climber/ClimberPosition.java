package frc.robot.subsystems.climber;

public enum ClimberPosition {
    BOTTOM(0.05),
    L1(4.39),
    CLIMB_L1(2.70);

    private final double height;

    ClimberPosition(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
