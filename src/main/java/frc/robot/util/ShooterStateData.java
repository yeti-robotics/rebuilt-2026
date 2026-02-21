package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

public class ShooterStateData {
    public static final Interpolator<ShooterStateData> interpolator = new Interpolator<ShooterStateData>() {
        @Override
        public ShooterStateData interpolate(ShooterStateData startValue, ShooterStateData endValue, double t) {
            double initialHoodPos = startValue.hoodPos;
            double finalHoodPos = endValue.hoodPos;
            double initialRPS = startValue.rps;
            double finalRPS = endValue.rps;
            double initialToF = startValue.timeOfFlight;
            double finalToF = endValue.timeOfFlight;

            return new ShooterStateData(
                    initialHoodPos + t * (finalHoodPos - initialHoodPos),
                    initialRPS + t * (finalRPS - initialRPS),
                    initialToF + t * (finalToF-initialToF));
        }

    };
    public final double hoodPos;
    public final double rps;
    public final double timeOfFlight;


    public ShooterStateData(double hoodPos, double rps, double timeOfFlight) {
        this.hoodPos = hoodPos;
        this.rps = rps;
        this.timeOfFlight = timeOfFlight;
    }
}
