package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorVoltage = 0;
        public double topMotorRPM = 0;

        public double bottomMotorVoltage = 0;
        public double bottomMotorRPM = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void spinMotors(double volts) {}

    public default void stopMotors() {}

    public default void shootFuel(){}
}
