package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public Command spinIndexer(double volts) {
        return run(() -> io.spinIndexerRoller(volts));
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
}
