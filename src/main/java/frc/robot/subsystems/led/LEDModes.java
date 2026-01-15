package frc.robot.subsystems.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDModes {
    RED_TO_BLUE_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDConstants.VIHAAN_RED, LEDConstants.YETI_BLUE)),
    BLUE_TO_RED_TRANSITION(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDConstants.YETI_BLUE, LEDConstants.VIHAAN_RED)),

    BLUE_ALLIANCE_ACTIVE(LEDPattern.solid(LEDConstants.YETI_BLUE)),
    RED_ALLIANCE_ACTIVE(LEDPattern.solid(new Color(197, 34, 51))),
    TRANSITION_ACTIVE(LEDPattern.solid(Color.kPurple)),
    ENDGAME_ACTIVE(LEDPattern.solid(Color.kGreen)),

    RAINBOW(LEDPattern.rainbow(255,128).scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), LEDConstants.LED_SPACING), true);

    public final LEDPattern pattern;
    public final boolean isAnimation;

    LEDModes(LEDPattern pattern) {
        this(pattern, false);
    }
    LEDModes(LEDPattern pattern, boolean isAnimation) {
        this.pattern = pattern;
        this.isAnimation = isAnimation;
    }
}
