package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerVoltage = 0;

        public boolean isDetected = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void spinIndexer(double volts) {}

    public default void applyPower(double power) {}
}
