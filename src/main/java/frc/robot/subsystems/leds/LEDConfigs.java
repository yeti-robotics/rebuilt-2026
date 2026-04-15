package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDConfigs {
    static final int CANDLE_ID = 0;

    CANdleConfiguration candleConfigs = new CANdleConfiguration().withCANdleFeatures(new CANdleFeaturesConfigs());
}
