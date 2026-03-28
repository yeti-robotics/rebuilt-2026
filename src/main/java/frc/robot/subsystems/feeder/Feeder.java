package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private FeederIO io;
    private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public Command runMotors(double rps) {
        return runEnd(() -> io.spinFeeder(rps), () -> io.spinFeeder(0));
    }

    public Command runMotorsUntilDetected(double volts) {
        return runEnd(() -> io.spinFeeder(volts), () -> io.spinFeeder(0)).until(() -> inputs.isDetected);
    }

    public Command feed(double rps) {
        return runMotors(rps).onlyIf(() -> inputs.isDetected);
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }

    public Command apply(double power) {
        return run(() -> io.applyPower(power));
    }

    public Command stop() {
        return runOnce(() -> io.applyPower(0));
    }

    public boolean canRangeDetected() {
        return inputs.isDetected;
    }
}
