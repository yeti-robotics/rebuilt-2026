package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOAlphaInputsAutoLogged inputs = new ShooterIOAlphaInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command shoot() {
        return run(() -> io.rollMotors(6));
    }
}
