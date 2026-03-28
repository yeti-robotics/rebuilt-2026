package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double feederVoltage = 0;
        public boolean isDetected = false;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void spinFeeder(double rps) {}

    public default void applyPower(double power) {}
}
