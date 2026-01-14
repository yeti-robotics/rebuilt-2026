package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDModes {
    RED_ALLIANCE_ACTIVE(LEDPattern.solid(new Color(197, 34, 51))),
    RED_TO_BLUE_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDConstants.VIHAAN_RED, LEDConstants.YETI_BLUE)),
    BLUE_TO_RED_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDConstants.YETI_BLUE, LEDConstants.VIHAAN_RED)),
    BLUE_ALLIANCE_ACTIVE(LEDPattern.solid(LEDConstants.YETI_BLUE));
    public final LEDPattern pattern;

    LEDModes(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
