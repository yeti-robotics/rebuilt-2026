package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearVelocity;

public enum ClimberState {
    DEFAULT(MetersPerSecond.of(8.5), RotationsPerSecond.of(2).in(RadiansPerSecond)),
    CLIMB(MetersPerSecond.of(8.5 / 4.0), RotationsPerSecond.of(1).in(RadiansPerSecond) / 4);

    public final LinearVelocity kSpeedAt12Volts;
    public final double MaFxAngularRate;

    ClimberState(LinearVelocity kSpeedAt12Volts, double MaFxAngularRate) {
        this.kSpeedAt12Volts = kSpeedAt12Volts;
        this.MaFxAngularRate = MaFxAngularRate;
    }

    public LinearVelocity kSpeedAt12Volts() {
        return kSpeedAt12Volts;
    }

    public double getAngularRate() {
        return MaFxAngularRate;
    }

    public ClimberState switchState() {
        return (this == DEFAULT || this == CLIMB) ? (this == DEFAULT ? CLIMB : DEFAULT) : this;
    }
}
