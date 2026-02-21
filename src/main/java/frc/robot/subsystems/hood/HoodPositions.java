package frc.robot.subsystems.hood;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum HoodPositions {
    STOW(0.0),
    BUMP(0.0),
    DEPLOY(1.0);

    private final Angle position;

    HoodPositions(double position) {
        this.position = Units.Rotations.of(position);
    }

    public Angle getPosition() {
        return position;
    }
}
