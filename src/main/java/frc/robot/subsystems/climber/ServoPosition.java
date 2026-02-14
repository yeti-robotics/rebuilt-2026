package frc.robot.subsystems.climber;

import edu.wpi.first.units.Units;

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
