package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int LED_STRIP_PORT = 0;
    public static final int LED_COUNT = 72;
    public static final Distance LED_SPACING = Meters.of(1 / 65.0);
    public static final Color YETI_BLUE = new Color(84, 182, 229);
    public static final double ZERO_TOLERANCE = 0.005;
}