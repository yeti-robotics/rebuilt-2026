package frc.robot.subsystems.powerdist;

import org.littletonrobotics.junction.AutoLog;

public interface PowerDistributorIO {
    @AutoLog
    public static class PowerDistributorIOInputs {
        public double totalCurrent;
        public double totalVoltage;
        public double[] channelCurrents;
        public double temperature;
    }

    public default void updateInputs(PowerDistributorIOInputs inputs) {}
}
