package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDsConfigs {
    public static final int CANDLE_ID = 25;
    public static final int LED_COUNT = 24;
    public static final int LED_START_COUNT = 8;

    public static final CANdleConfiguration CANDLE_CONFIGS = new CANdleConfiguration()
            .withLED(new LEDConfigs()
                    .withStripType(StripTypeValue.BRG)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs)
                    .withBrightnessScalar(1.0));

    public static final LarsonAnimation defaultAnim =
            new LarsonAnimation(LED_START_COUNT, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(84, 182, 229));
}
