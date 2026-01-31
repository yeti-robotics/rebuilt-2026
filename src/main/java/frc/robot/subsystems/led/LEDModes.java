package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDModes {
    BLUE_ALLIANCE_ACTIVE(LEDPattern.solid(LEDConstants.YETI_BLUE)),
    RED_ALLIANCE_ACTIVE(LEDPattern.solid(LEDConstants.VIHAAN_RED)),
    RAINBOW(
            LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), LEDConstants.LED_SPACING),
            true),
    TRANSITION_ACTIVE(LEDPattern.solid(Color.kPurple)),
    ENDGAME_ACTIVE(LEDPattern.solid(Color.kGreen)),
    LOCKED_GREEN(LEDPattern.solid(LEDConstants.BRIGHT_GREEN).blink(Seconds.of(0.25)), true),
    BLINKING_ORANGE(LEDPattern.solid(Color.kOrange).synchronizedBlink(RobotController::getRSLState), true),
    NOT_LOCKED_RED(LEDPattern.solid(Color.kRed)),
    SNOWFALL(new SnowfallLEDPattern(), true),
    WAVE(new WaveLEDPattern(), true),
    ALLIANCE_DEACTIVATION_WARNING(
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, LEDConstants.VIHAAN_RED, LEDConstants.YETI_BLUE)
                    .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), LEDConstants.LED_SPACING),
            true);

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
