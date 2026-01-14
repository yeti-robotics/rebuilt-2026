package frc.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ClimberPosition {
    BOTTOM(0.0),
    TOWER1(0),
    TOWER2(0),
    TOWER3(0);

    private final Angle height;

    ClimberPosition(double height) {
        this.height = Units.Rotations.of(height);
    }
}
