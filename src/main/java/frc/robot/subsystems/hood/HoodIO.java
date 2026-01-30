package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double hoodPosition = 0.0;
        public double hoodVelocity = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void moveToPosition(double position) {}

    default void testSpinHopperRoller(double percent) {}
}
