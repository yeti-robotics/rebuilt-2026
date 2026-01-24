package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO;

import java.util.logging.Logger;

import static edu.wpi.first.hal.simulation.PWMDataJNI.setPosition;

public class HoodSubsystem extends SubsystemBase {
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public HoodSubsystem(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic(HoodIO io) {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", hoodInputs);
    }

    public double getHoodPosition() {
        return inputs.hoodPosition;
    }

    public double getHoodVelocity() {
        return inputs.hoodVelocity;
    }

    public void moveToPosition(double position) {
       moveToPosition(position);
    }


}
