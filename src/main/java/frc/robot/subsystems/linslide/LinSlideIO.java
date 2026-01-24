package frc.robot.subsystems.linslide;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface LinSlideIO {
    @AutoLog
    public static class LinSlideIOInputs {
        public double positionRotation = 0.0;
        public double targetPositionRotation = 0.0;
        public boolean isDeployed = false;
    }

    public default void updateInputs(LinSlideIOInputs inputs) {}

    public default void moveToPosition(Angle position) {}

    public default void zeroPosition() {}

    public default void applyPower(double percent) {}
}
