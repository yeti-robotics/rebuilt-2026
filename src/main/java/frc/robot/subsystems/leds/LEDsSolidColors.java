package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

public enum LEDsSolidColors {
    COLIN_WHITE(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(255, 255, 255))),
    BHANU_MAROON(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(120, 10, 7))),
    ANISH_GIRLYPOP_PINK(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(255, 4, 27))),
    VIHAAN_RED(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(255, 0, 0))),
    NICK_ORANGE(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(255, 56, 2))),
    DANYA_GREEN(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(144, 238, 144))),
    VIVEK_LIME(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(220, 255, 9))),
    AMIT_TEAL(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(2, 255, 87))),
    ANIKA_CORNFLOWER(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(98, 159, 252))),
    YETI_BLUE(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(84, 182, 229))),
    BRENNEN_MIDNIGHT(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(52, 18, 238))),
    MUKIE_PURPLE(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(235, 59, 255))),
    FIONN_VIOLET(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(53, 6, 62)));

    private final SolidColor color;

    LEDsSolidColors(SolidColor color) {
        this.color = color;
    }

    public SolidColor getColor() {
        return color;
    }
}
