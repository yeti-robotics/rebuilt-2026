package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public Command setRoller(double volts) {
        return startEnd(() -> io.setIntakeMotorDuty(volts), () -> io.setIntakeMotorDuty(0));
    }
}
