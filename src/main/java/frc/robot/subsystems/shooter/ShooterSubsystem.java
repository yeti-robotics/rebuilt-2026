package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterStateData;
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

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }

    public static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
        InterpolatingTreeMap<Double, ShooterStateData> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

        map.put(0.0, new ShooterStateData(0,0, 0));

        return map;
    }
}
