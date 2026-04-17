package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDsConfigs {
    public static final int CANDLE_ID = 0;
    private final int LED_COUNT = 100;

    public static final CANdleConfiguration CANDLE_CONFIGS = new CANdleConfiguration()
            .withLED(new LEDConfigs()
                    .withStripType(StripTypeValue.RGB)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
                    .withBrightnessScalar(1.0));
}
