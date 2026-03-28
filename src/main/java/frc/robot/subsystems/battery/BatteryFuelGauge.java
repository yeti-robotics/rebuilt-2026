package frc.robot.subsystems.battery;

import com.playingwithfusion.BattFuelGauge;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class BatteryFuelGauge extends SubsystemBase {
    interface BatteryFuelGaugeIO {
        @AutoLog
        class BatteryFuelGaugeInputs {
            float voltage = 0;
            float current = 0;
            float charge = 0;
        }
    }

    private BattFuelGauge batteryGauge;
    public BatteryFuelGaugeInputsAutoLogged inputs = new BatteryFuelGaugeInputsAutoLogged();

    public BatteryFuelGauge(int ID) {
        batteryGauge = new BattFuelGauge(ID);
        batteryGauge.setManufacturer(BattFuelGauge.BatteryManufacturer.MKPowered);
    }

    public void updateInputs(BatteryFuelGaugeIO.BatteryFuelGaugeInputs inputs) {
        inputs.voltage = batteryGauge.getVoltage();
        inputs.current = batteryGauge.getCurrent();
        inputs.charge = batteryGauge.getRemainingChargePct();
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Battery Gauge", inputs);
    }

    public void saveLog() {
        //try {
          File file = new File("D:\\logs\\BatteryData.csv");
            FileWriter fw = new FileWriter(file, true);
            BufferedWriter out = new BufferedWriter(fw);
        //} catch (IOException e) { e.printStackTrace(); }
    }
}
