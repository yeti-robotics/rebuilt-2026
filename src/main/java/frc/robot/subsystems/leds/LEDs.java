package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDs extends SubsystemBase {
    private final CANdle candle = new CANdle(LEDsConfigs.CANDLE_ID, Constants.rioBus);

    public LEDs() {
        candle.getConfigurator().apply(LEDsConfigs.CANDLE_CONFIGS);
    }

    public void setAnimation(Animation0TypeValue type) {
        switch (type) {
            case Empty:
                candle.setControl(new EmptyAnimation(0));
                break;
            case ColorFlow:
                candle.setControl(new ColorFlowAnimation(0, 7));
                break;
            case Fire:
                candle.setControl(new FireAnimation(0, 7));
                break;
            case Larson:
                candle.setControl(new LarsonAnimation(0, 7));
                break;
            case Rainbow:
                candle.setControl(new RainbowAnimation(0, 7));
                break;
            case RgbFade:
                candle.setControl(new RgbFadeAnimation(0, 7));
                break;
            case SingleFade:
                candle.setControl(new SingleFadeAnimation(0, 7));
                break;
            case Strobe:
                candle.setControl(new StrobeAnimation(0, 7));
                break;
            case Twinkle:
                candle.setControl(new TwinkleAnimation(0, 7));
                break;
            case TwinkleOff:
                candle.setControl(new TwinkleOffAnimation(0, 7));
                break;
        }
    }
}
