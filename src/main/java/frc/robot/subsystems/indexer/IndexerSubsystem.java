package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    public boolean sensorGetIsDetected() {
        return inputs.isDetected;
    }

    public Command index(double volts) {
        return runEnd(() -> io.spinIndexer(volts), () -> io.spinIndexer(0))
                .withTimeout(0.5)
                .andThen(runEnd(() -> io.spinIndexer(volts), () -> io.spinIndexer(0))
                        .unless(this::sensorGetIsDetected));
    }

    public Command runMotors(double volts) {
        return runEnd(() -> io.spinIndexer(volts), () -> io.spinIndexer(0));
    }
}
