package frc.robot.subsystems.powerdist;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributorPDH implements PowerDistributorIO {
    private final PowerDistribution pdh;

    public PowerDistributorPDH() {
        pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    }

    @Override
    public void updateInputs(PowerDistributorIOInputs inputs) {
        inputs.totalCurrent = pdh.getTotalCurrent();
        inputs.totalVoltage = pdh.getVoltage();
        inputs.channelCurrents = pdh.getAllCurrents();
        inputs.temperature = pdh.getTemperature();
    }
}
