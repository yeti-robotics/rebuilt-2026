package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double rollerSpeed = 0.0;
        public boolean isFull = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void spinIndexerRoller(double volts) {}

    public default void applyPower(double volts) {}
}
