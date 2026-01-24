package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public AngularVelocity getVelocity() {
        return Units.RotationsPerSecond.of(inputs.topMotorRPM);
    }

    public Command shoot(double velocity) {
        return startEnd(() -> io.spinMotors(velocity), () -> io.stopMotors());
    }
}
