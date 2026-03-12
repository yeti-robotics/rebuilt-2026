package frc.robot.subsystems.battery;

import com.playingwithfusion.BattFuelGauge;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class BatteryFuelGauge extends SubsystemBase {
    @AutoLog
    public static class BatteryFuelGaugeInputs {
        float voltage = 0;
        float current = 0;
        float charge = 0;
    }

    private BattFuelGauge batteryGauge;
    public BatteryFuelGaugeInputsAutoLogged inputs = new BatteryFuelGaugeInputsAutoLogged();

    public BatteryFuelGauge(int ID) {
        batteryGauge = new BattFuelGauge(ID);
        batteryGauge.setManufacturer(BattFuelGauge.BatteryManufacturer.Duracell);

        batteryGauge.saveLog("");
    }

    public void updateInputs(BatteryFuelGaugeInputs inputs) {
        inputs.voltage = batteryGauge.getVoltage();
        inputs.current = batteryGauge.getCurrent();
        inputs.charge = batteryGauge.getRemainingChargePct();
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Battery Gauge", inputs);
    }
}
