package frc.robot.subsystems.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystemAlpha extends SubsystemBase {
    private ArmIOAlpha io;
    private ArmIOAlphaInputsAutoLogged inputs = new ArmIOAlphaInputsAutoLogged();

    public ArmSubsystemAlpha(ArmIOAlpha io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public Command moveToPosition(Angle position) {
        return runOnce(() -> io.moveToPosition(position));
    }

    public double getCurrentPosition() {
        return inputs.positionRotation;
    }

    public double getTargetPosition() {
        return inputs.targetPositionRotation;
    }
}
