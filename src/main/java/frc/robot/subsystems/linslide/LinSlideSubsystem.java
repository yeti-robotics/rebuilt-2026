package frc.robot.subsystems.linslide;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LinSlideSubsystem extends SubsystemBase {
    private LinSlideIO io;
    private LinSlideIOInputsAutoLogged inputs = new LinSlideIOInputsAutoLogged();

    public LinSlideSubsystem(LinSlideIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LinSlide", inputs);
    }

    public Command moveToPosition(double power, boolean target) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0))
                .until(() -> target ? inputs.isDeployed : inputs.isStowed);
    }

    public Command applyPower(double power) {
        return runOnce(() -> io.applyPower(power));
    }

    public Command zero() {
        return runOnce(() -> io.zeroPosition());
    }

    public double getCurrentPosition() {
        return inputs.positionRotation;
    }

    public double getTargetPosition() {
        return inputs.targetPositionRotation;
    }

    public boolean isDeployed() {
        return inputs.isDeployed;
    }
}
