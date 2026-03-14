package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int LED_STRIP_PORT = 0;
    public static final int LED_COUNT = 36;
    public static final Distance LED_SPACING = Meters.of(0.01);
    public static final Color YETI_BLUE = new Color(84, 229, 182);
    public static final Color VIHAAN_RED = new Color(197, 51, 34);
    public static final double ANIMATION_TIMEOUT_SECONDS = 5.0;
    public static final Color BRIGHT_GREEN = new Color(0, 0, 255);
}
