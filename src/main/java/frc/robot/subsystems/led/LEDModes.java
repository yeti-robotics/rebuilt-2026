package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDModes {
    RED_ALLIANCE_ACTIVE(LEDPattern.solid(Color.kRed)),
    BLUE_ALLIANCE_ACTIVE(LEDPattern.solid(new Color(84, 182, 229)));

    public final LEDPattern pattern;

    LEDModes(LEDPattern pattern) {
        this.pattern = pattern;
    }
}
