package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIOAlpha {
    @AutoLog
    public static class ArmIOAlphaInputs {
        public double positionRotation = 0.0;
        public double targetPositionRotation = 0.0;
    }

    public default void updateInputs(ArmIOAlphaInputs inputs) {}

    public default void moveToPosition(Angle position) {}
}
