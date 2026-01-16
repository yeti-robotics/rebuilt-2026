package frc.robot.subsystems.linslide;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum LinSlidePosition {
    STOW(0.0),
    DEPLOY(0.75);

    private final Angle position;

    LinSlidePosition(double position) {
        this.position = Units.Rotations.of(position);
    }

    public Angle getPosition() {
        return position;
    }
}
