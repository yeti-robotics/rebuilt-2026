package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
        new Trigger(() -> inputs.isAtBottom)
                .onTrue(Commands.runOnce(io::zeroPosition).andThen(io::neutralizeClimber));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public boolean climberBottomDetection() {
        return inputs.isAtBottom;
    }

    public double getCurrentPosition() {
        return inputs.position;
    }

    public double getTargetPosition() {
        return inputs.targetPosition;
    }

    public Command moveToPosition(Angle position) {
        return run(() -> io.setClimberPosition(position));
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0));
    }

    public Command extendServo() {
        return run(() -> io.setAngle(ServoPosition.SERVO_UPPER.getDegrees()));
    }

    public Command stowServo() {
        return run(() -> io.setAngle(ServoPosition.SERVO_LOWER.getDegrees()));
    }

    public Command setAngle(double position) {
        return run(() -> io.setAngle(position));
    }

    public Command deploy(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0)).until(() -> getCurrentPosition() > 4.39);
    }

    public Command stow(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0)).until(() -> getCurrentPosition() < 0.05);
    }

    public Command climb(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0)).until(() -> getCurrentPosition() < 2.70);
    }
}
