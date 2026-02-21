package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public HoodSubsystem(HoodIO io) {
        this.io = io;
        setDefaultCommand(stowHood().onlyIf(() -> getHoodPosition() > 0.1));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public double getHoodPosition() {
        return inputs.hoodPosition;
    }

    public double getHoodVelocity() {
        return inputs.hoodVelocity;
    }

    public Command deployHood() {
        return runOnce(() -> io.moveToPosition(HoodPositions.DEPLOY.getPosition()));
    }

    public Command stowHood() {
        return runOnce(() -> io.moveToPosition(HoodPositions.STOW.getPosition()));
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }

    public static Angle calcPos(double distance) {
        return ShooterSubsystem.SHOOTER_MAP().get(distance).hoodPos;
    }

    public void moveTo(Angle position) {
        io.moveToPosition(position);
    }
}
