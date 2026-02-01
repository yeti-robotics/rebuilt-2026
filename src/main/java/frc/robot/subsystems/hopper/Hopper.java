package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public Command spinHopper(double volts) {
        return run(() -> io.spinHopperRoller(volts));
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }
}
