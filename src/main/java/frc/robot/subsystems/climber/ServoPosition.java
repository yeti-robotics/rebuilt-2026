package frc.robot.subsystems.climber;

public enum ServoPosition {
    SERVO_UPPER(135),
    SERVO_LOWER(45);

    private final double degrees;

    ServoPosition(double degrees) {
        this.degrees = degrees;
    }

    public double getDegrees() {
        return degrees;
    }
}
