package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double hoodPosition = 0.0;
        public double hoodTargetPosition = 0.0;
        public double hoodVelocity = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void moveToPosition(Angle position) {}

    default void applyPower(double percent) {}
}
