package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean isAtBottom = true;
        public double position = 0.0;
        public double targetPosition = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberPosition(Angle position) {}

    public default void zeroPosition() {}

    public default void neutralizeClimber() {}

    public default void applyPower(double percent) {}

    public default void setAngle(double position) {}
}
