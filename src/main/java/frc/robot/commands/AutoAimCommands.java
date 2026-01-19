package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

import java.util.function.DoubleSupplier;

public class AutoAimCommands {
    private static final PIDController headingController = new PIDController(5, 0, 0);

    public static Command autoAimWithOrbit(
            Drive drive,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {

        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return drive.runEnd(
                () -> {
                    Pose2d currentPose = drive.getPose();
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = currentPose.getRotation();

                    Translation2d hubDistance = modifiedTarget.minus(currentPosition);

                    double rawXVelo = xVelSupplier.getAsDouble();
                    double rawYVelo = yVelSupplier.getAsDouble();

                    Rotation2d targetHeading = hubDistance.getAngle().plus(Rotation2d.kPi); // remove if needed for real robot
                    Translation2d fieldRel = new Translation2d(rawXVelo, rawYVelo).rotateBy(targetHeading);

                    double angularVelo = headingController.calculate(
                            currentRotation.getRadians(),
                            targetHeading.getRadians());

                    ChassisSpeeds currentReference = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRel.getX(),
                            fieldRel.getY(),
                            angularVelo,
                            currentRotation);

                    drive.runVelocity(currentReference);
                },
                drive::stop);
    }
}
