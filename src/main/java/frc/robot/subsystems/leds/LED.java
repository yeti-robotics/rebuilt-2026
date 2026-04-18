package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.sim.CANdleSimState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(LEDsConfigs.CANDLE_ID, Constants.rioBus);
    private final CANdleSimState candleSim = candle.getSimState();

    public LED() {
        candle.getConfigurator().apply(LEDsConfigs.CANDLE_CONFIGS);
        // setDefaultCommand(runOnce(() -> candle.setControl(LEDsConfigs.defaultAnim)));
    }

    public void setAnimation(Animation0TypeValue type) {
        switch (type) {
            case Empty:
                candle.setControl(new EmptyAnimation(0));
                break;
            case ColorFlow:
                candle.setControl(new ColorFlowAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case Fire:
                candle.setControl(new FireAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case Larson:
                candle.setControl(new LarsonAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case Rainbow:
                candle.setControl(new RainbowAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case RgbFade:
                candle.setControl(new RgbFadeAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case SingleFade:
                candle.setControl(new SingleFadeAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case Strobe:
                candle.setControl(new StrobeAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case Twinkle:
                candle.setControl(new TwinkleAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
            case TwinkleOff:
                candle.setControl(new TwinkleOffAnimation(7, LEDsConfigs.LED_COUNT - 1));
                break;
        }
    }

    public void setLEDs(int startIndex, int count, int r, int g, int b) {
        candle.setControl(new ColorFlowAnimation(startIndex, startIndex + count - 1)
                .withSlot(0)
                .withColor(new RGBWColor(r, g, b, 0))
                .withDirection(AnimationDirectionValue.Forward)
                .withUpdateFreqHz(0));
    }

    public Command setSolidColor(SolidColor solidColorName) {
        return runOnce(this::clearLEDs)
                .andThen(runOnce(this::clearSolidColor))
                .andThen(run(() -> candle.setControl(solidColorName)));
    }

    public void setColor(int start, int end, Color8Bit color, double brightness) {
        candle.setControl(new SolidColor(start, end)
                .withColor(new RGBWColor(color.red, color.green, color.blue).scaleBrightness(brightness)));
    }

    public void clearLEDs() {
        candle.setControl(new EmptyAnimation(0));
    }

    public void clearSolidColor() {
        candle.setControl(new SolidColor(7, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(0, 0, 0)));
    }

    public void defaultColor() {
        candle.setControl(new LarsonAnimation(0, LEDsConfigs.LED_COUNT - 1).withColor(new RGBWColor(84, 182, 229)));
    }

    @Override
    public void simulationPeriodic() {
        candleSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
