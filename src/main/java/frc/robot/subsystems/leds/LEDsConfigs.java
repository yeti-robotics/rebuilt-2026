package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDsConfigs {
    public static final int CANDLE_ID = 25;
    private final int LED_COUNT = 100;

    public static final CANdleConfiguration CANDLE_CONFIGS = new CANdleConfiguration()
            .withLED(new LEDConfigs()
                    .withStripType(StripTypeValue.BRG)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
                    .withBrightnessScalar(1.0));
}
