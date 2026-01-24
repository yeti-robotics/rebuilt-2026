package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public HoodSubsystem(HoodIO io) {
        this.io = io;
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

    public void moveToPosition(double position) { io.moveToPosition(position);}
}
