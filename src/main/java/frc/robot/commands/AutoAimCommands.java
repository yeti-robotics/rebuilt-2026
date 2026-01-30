package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;

public class AutoAimCommands {
    public static final PIDController headingController = new PIDController(5, 0, 0);

    private static final double SPEED_MULTIPLIER = 3;

    static {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command autoAimWithOrbit(
            CommandSwerveDrivetrain drive,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);

        return drive.runEnd(
                () -> {
                    Pose2d currentPose = drive.getState().Pose;
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = currentPose.getRotation();

                    Translation2d hubDistance = modifiedTarget.minus(currentPosition);

                    double rawXVelo = -xVelSupplier.getAsDouble() * SPEED_MULTIPLIER;
                    double rawYVelo = -yVelSupplier.getAsDouble() * SPEED_MULTIPLIER;

                    Rotation2d targetHeading =
                            hubDistance.getAngle().plus(Rotation2d.kPi); // remove if needed for real robot
                    Translation2d fieldRel = new Translation2d(rawXVelo, rawYVelo).rotateBy(targetHeading);

                    double angularVelo =
                            headingController.calculate(currentRotation.getRadians(), targetHeading.getRadians());

                    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                            .withVelocityX(rawXVelo)
                            .withVelocityY(rawYVelo)
                            .withRotationalRate(angularVelo)
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

                    drive.run(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRel.getX(), fieldRel.getY(), angularVelo, currentRotation));

                    drive.setControl(request);
                },
                SwerveRequest.Idle::new);
    }

    private static double calculateAngularVelocity(Pose2d currentPose, Translation2d target) {
        if (target == null) {
            return 0;
        }

        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d desiredHeading = target.minus(currentPose.getTranslation())
                .getAngle()
                .rotateBy(Rotation2d.k180deg); // Remove this .rotateBy() if needed for real bot

        return headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());
    }

    private static Rotation2d calcDesiredHeading(Pose2d currentPose, Translation2d target) {
        if (target == null) {
            return Rotation2d.kZero;
        }

        Rotation2d desiredHeading = target.minus(currentPose.getTranslation())
                .getAngle()
                .rotateBy(Rotation2d.k180deg); // Remove this .rotateBy() if needed for real bot

        return desiredHeading;
    }

    public static Command autoAim(
            CommandSwerveDrivetrain drive,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);

        return drive.runEnd(
                () -> {
                    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                            .withHeadingPID(5, 0, 0)
                                    .withVelocityX(-xVelSupplier.getAsDouble() * SPEED_MULTIPLIER)
                                            .withVelocityY(-yVelSupplier.getAsDouble() * SPEED_MULTIPLIER)
                                                    .withTargetDirection(calcDesiredHeading(drive.getState().Pose, modifiedTarget))
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

                    drive.setControl(request);
                },
                SwerveRequest.Idle::new);
    }
}
