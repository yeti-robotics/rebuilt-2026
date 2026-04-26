package frc.robot.subsystems.linslide;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LinSlide extends SubsystemBase {
    private LinSlideIO io;
    private LinSlideIOInputsAutoLogged inputs = new LinSlideIOInputsAutoLogged();

    public LinSlide(LinSlideIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LinSlide", inputs);
    }

    public Command runIntake(double power, boolean stow) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0))
                .until(() -> stow ? inputs.isDeployed : inputs.isStowed);
    }

    public Command agitate() {
        return applyPower(0.2)
                .until(() -> inputs.positionRotation
                        == LinSlidePosition.HALF.getPosition().magnitude())
                .andThen(applyPower(-0.2)
                        .until(() -> inputs.positionRotation
                                == LinSlidePosition.DEPLOY.getPosition().magnitude()))
                .repeatedly();
    }

    public Command applyPower(double power) {
        return runEnd(() -> io.applyPower(power), () -> io.applyPower(0))
                .until(() -> Units.RotationsPerSecond.of(inputs.velocityRPM).isNear(Units.RotationsPerSecond.of(0), 0.1)
                        && inputs.supplyCurrent > 10);
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

    public boolean isBasicallyZeroRPM() {
        return Units.RotationsPerSecond.of(inputs.velocityRPM).isNear(Units.RotationsPerSecond.of(0), 0.2);
    }

    public boolean isCloseToZero() {
        return Rotations.of(inputs.positionRotation).isNear(Rotations.of(0), 0.05);
    }

    public Command defaultMovement(double volts) {
        return run(() -> io.applyPower(volts)).until(this::isBasicallyZeroRPM).repeatedly();
    }

    public Command setLinslidePosition(double position) {
        return runOnce(() -> io.setPosition(position));
    }
}
