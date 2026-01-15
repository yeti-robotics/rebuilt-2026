package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int LED_STRIP_PORT = 0;
    public static final int LED_COUNT = 72;
    public static final Distance LED_SPACING = Meters.of(0.01);
    public static final Color YETI_BLUE = new Color(84, 182, 229);
    public static final Color VIHAAN_RED = new Color(197, 34, 51);
}