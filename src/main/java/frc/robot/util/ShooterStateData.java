package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

public class ShooterStateData {
    public static final Interpolator<ShooterStateData> interpolator = new Interpolator<ShooterStateData>() {
        @Override
        public ShooterStateData interpolate(ShooterStateData startValue, ShooterStateData endValue, double t) {
            double initialRPS = startValue.rps;
            double finalRPS = endValue.rps;
            double initialToF = startValue.timeOfFlight;
            double finalToF = endValue.timeOfFlight;

            return new ShooterStateData(
                    initialRPS + t * (finalRPS - initialRPS),
                    initialToF + t * (finalToF-initialToF));
        }

    };
    public final double rps;
    public final double timeOfFlight;


    public ShooterStateData(double rps, double timeOfFlight) {
        this.rps = rps;
        this.timeOfFlight = timeOfFlight;
    }
}
