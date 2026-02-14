package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public boolean isAtSpeed() {
        return io.isAtSpeed(targetSpeed);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }
}
