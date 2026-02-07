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

    public Command setIntake(double volts) {
        return startEnd(() -> io.setIntakeMotor(volts), () -> io.setIntakeMotor(0));
    }

    public Command rollIn() {
        return setIntake(IntakeConfigs.INTAKE_VOLTAGE);
    }

    public Command rollOut() {
        return setIntake(IntakeConfigs.OUTTAKE_VOLTAGE);
    }

    public Command applyPower(double percent) {
        return runEnd(() -> io.applyPower(percent), () -> io.applyPower(0));
    }

    public Command setRollerSim(double power) {
        return runOnce(() -> io.setRunning(true))
                .andThen(runOnce(() -> System.out.println("setRunning is true")))
                .andThen(runEnd(
                        () -> {
                            io.setIntakeMotor(power);
                            System.out.println("Intake");
                        },
                        () -> {
                            io.setIntakeMotor(0);
                            io.setRunning(false);
                        }))
                .andThen(() -> io.setRunning(false))
                .andThen(runOnce(() -> System.out.println("setRunning is false")));
    }

    public Command handoffFuel() {
        return runOnce(io::handoffFuel);
    }
}
