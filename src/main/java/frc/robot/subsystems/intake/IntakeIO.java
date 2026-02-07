package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double alphaMotorRPM = 0.0;
        public double alphaMotorVoltage = 0.0;
        public double primaryMotorRPM = 0.0;
        public double primaryMotorVoltage = 0.0;
        public double secondaryMotorRPM = 0.0;
        public double secondaryMotorVoltage = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeMotor(double volts) {}

    public default void applyPower(double percent) {}
}
