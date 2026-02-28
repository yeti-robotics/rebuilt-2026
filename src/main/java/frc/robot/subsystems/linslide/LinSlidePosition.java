package frc.robot.subsystems.linslide;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum LinSlidePosition {
    STOW(0.0),
    HALF(0.16),
    DEPLOY(0.32);

    private final Angle position;

    LinSlidePosition(double position) {
        this.position = Units.Rotations.of(position);
    }

    public Angle getPosition() {
        return position;
    }
}
