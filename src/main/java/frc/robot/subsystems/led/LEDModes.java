package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDModes {
    RED_ALLIANCE_ACTIVE(LEDPattern.solid(new Color(197, 34, 51))),
    RED_TO_BLUE_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(197, 34, 51), new Color(84, 182, 229))),
    BLUE_TO_RED_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(84, 182, 229), new Color(197, 34, 51))),
    BLUE_ALLIANCE_ACTIVE(LEDPattern.solid(new Color(84, 182, 229)));
    public final LEDPattern pattern;

    LEDModes(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
