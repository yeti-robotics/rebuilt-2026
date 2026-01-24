package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerRPM = 0.0;
        public double rollerVoltage = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeMotor(double volts) {}

    public default void setRunning(boolean runIntake) {}

    public default boolean isFuelInsideIntake(boolean isFuelInsideIntake) {}

}
