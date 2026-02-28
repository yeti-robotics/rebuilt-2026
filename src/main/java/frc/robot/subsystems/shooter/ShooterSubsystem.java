package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodPositions;
import frc.robot.util.ShooterStateData;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetSpeed = 0;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Target Speed", targetSpeed);
        Logger.recordOutput("Shooter/Is At Speed", isAtSpeed());
    }

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public AngularVelocity getVelocity() {
        return Units.RotationsPerSecond.of(inputs.topMotorRPM);
    }

    public Command shoot(double velocity) {
        return runOnce(() -> this.targetSpeed = velocity)
                .andThen(runEnd(() -> io.spinMotors(velocity), () -> io.stopMotors()));
    }

    public Command shootForever(double velocity) {
        return runOnce(() -> this.targetSpeed = velocity).andThen(run(() -> io.applyPower(velocity)));
    }

    public Command revUpFlywheels(double velocity) {
        return runOnce(() -> this.targetSpeed = velocity).andThen(run(() -> io.spinMotors(velocity)));
    }

    public Command stopFlywheels() {
        return runOnce(() -> io.stopMotors()).andThen(() -> this.targetSpeed = 0);
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }

    public static final InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        SHOOTER_MAP.put(
                2.08, new ShooterStateData(HoodPositions.STOW.getPosition(), 25, 0.0)); // hood, rps, flight time
        //        SHOOTER_MAP.put(3.3, new ShooterStateData(HoodPositions.STOW.getPosition(), 28, 0.0));
        SHOOTER_MAP.put(3.38, new ShooterStateData(HoodPositions.STOW.getPosition(), 33, 0.0));
        SHOOTER_MAP.put(4.31, new ShooterStateData(HoodPositions.STOW.getPosition(), 35, 0.0));
        SHOOTER_MAP.put(2.58, new ShooterStateData(HoodPositions.STOW.getPosition(), 27, 0.0));
        SHOOTER_MAP.put(3.74, new ShooterStateData(HoodPositions.STOW.getPosition(), 34, 0.0));
        SHOOTER_MAP.put(4.46, new ShooterStateData(HoodPositions.STOW.getPosition(), 42, 0.0));
    }

    public boolean isAtSpeed() {
        return io.isAtSpeed(targetSpeed);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }
}
