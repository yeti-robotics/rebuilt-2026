package frc.robot.subsystems.linslide;

import org.littletonrobotics.junction.AutoLog;

public interface LinSlideIO {
    @AutoLog
    public static class LinSlideIOInputs {
        public double positionRotation = 0.0;
        public double targetPositionRotation = 0.0;
        public double velocityRPM = 0.0;
        public boolean isDeployed = false;
        public boolean isStowed = true;
    }

    public default void updateInputs(LinSlideIOInputs inputs) {}

    public default void moveToPosition(double dutySpeed) {}

    public default void zeroPosition() {}

    public default void applyPower(double percent) {}

    public default void defaultCommand(double volts) {}
}
