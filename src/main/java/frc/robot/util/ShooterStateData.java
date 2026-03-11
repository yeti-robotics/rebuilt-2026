package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.Angle;

public class ShooterStateData {

    public static final Interpolator<ShooterStateData> interpolator = (startValue, endValue, t) -> {
        double initialHood = startValue.hoodPos.in(Radians);
        double finalHood = endValue.hoodPos.in(Radians);

        double initialRPS = startValue.rps;
        double finalRPS = endValue.rps;

        double initialToF = startValue.timeOfFlight;
        double finalToF = endValue.timeOfFlight;

        double interpolatedHood = initialHood + t * (finalHood - initialHood);

        return new ShooterStateData(
                Radians.of(interpolatedHood),
                initialRPS + t * (finalRPS - initialRPS),
                initialToF + t * (finalToF - initialToF));
    };
    public final Angle hoodPos;
    public final double rps;
    public final double timeOfFlight;

    public ShooterStateData(Angle hoodPos, double rps, double timeOfFlight) {
        this.hoodPos = hoodPos;
        this.rps = rps;
        this.timeOfFlight = timeOfFlight;
    }
}
