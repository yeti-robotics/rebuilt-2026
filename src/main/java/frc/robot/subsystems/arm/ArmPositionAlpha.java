package frc.robot.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ArmPositionAlpha {
    STOW(0.0),
    DEPLOY(0.75);

    private final Angle position;

    ArmPositionAlpha(double position) {
        this.position = Units.Rotations.of(position);
    }

    public Angle getPosition() {
        return position;
    }
}
