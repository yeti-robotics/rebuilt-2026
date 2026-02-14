package frc.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import javax.swing.*;

public enum ClimberPosition {
    BOTTOM(0.0),
    L1(29.0),
    L2(47.0),
    L3(65.0),
    SERVO_UPPER(135),
    SERVO_LOWER(45);

    private final Angle height;

    ClimberPosition(double height) {
        this.height = Units.Rotations.of(height);
    }

    public Angle getHeight() {
        return height;
    }
}
