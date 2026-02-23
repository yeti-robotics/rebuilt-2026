package frc.robot.commands;

import static frc.robot.constants.Constants.currentMode;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstantsAlpha;
import frc.robot.subsystems.drive.TunerConstantsBeta;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAimCommands {
    public static final PIDController headingController = new PIDController(20, 0, 0);

    private static final double SPEED_MULTIPLIER = currentMode == Constants.Mode.ALPHA
            ? TunerConstantsAlpha.kSpeedAt12Volts.magnitude()
            : TunerConstantsBeta.kSpeedAt12Volts.magnitude();

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

                    double rawXVelo = xVelSupplier.getAsDouble() * SPEED_MULTIPLIER;
                    double rawYVelo = yVelSupplier.getAsDouble() * SPEED_MULTIPLIER;

                    Rotation2d targetHeading = hubDistance
                            .getAngle()
                            .plus(Rotation2d.kPi)
                            .rotateBy(AllianceFlipUtil.apply(Rotation2d.kZero));
                    ; // remove if needed for real robot
                    Translation2d fieldRel = new Translation2d(rawXVelo, rawYVelo).rotateBy(targetHeading);

                    double angularVelo =
                            headingController.calculate(currentRotation.getRadians(), targetHeading.getRadians());

                    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                            .withVelocityX(fieldRel.getX())
                            .withVelocityY(fieldRel.getY())
                            .withRotationalRate(angularVelo)
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

                    drive.setControl(request);
                },
                SwerveRequest.Idle::new);
    }

    private static Rotation2d calcDesiredHeading(Pose2d currentPose, Translation2d target) {
        if (target == null) {
            return Rotation2d.kZero;
        }

        Translation2d targetPose = AllianceFlipUtil.apply(target);
        Rotation2d desiredHeading = targetPose
                .minus(currentPose.getTranslation())
                .getAngle()
                .rotateBy(AllianceFlipUtil.apply(Rotation2d.kZero));

        Logger.recordOutput("AutoAim/Target Heading", desiredHeading);
        Logger.recordOutput("AutoAim/Target Pose", targetPose);

        return desiredHeading;
    }

    public static Command autoAim(
            CommandSwerveDrivetrain drive,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {

        return drive.runEnd(
                () -> {
                    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                            .withHeadingPID(20, 0, 0)
                            .withVelocityX(-xVelSupplier.getAsDouble() * SPEED_MULTIPLIER)
                            .withVelocityY(-yVelSupplier.getAsDouble() * SPEED_MULTIPLIER)
                            .withTargetDirection(calcDesiredHeading(drive.getState().Pose, target))
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

                    drive.setControl(request);
                },
                SwerveRequest.Idle::new);
    }

    public static Command readyAim(
            CommandSwerveDrivetrain drive,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            Translation2d target) {

        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        ShooterStateData state = ShooterSubsystem.SHOOTER_MAP.get(distance);
        double timeOfFlight = state.timeOfFlight;

        double joystickVX = -xVelSupplier.getAsDouble() * SPEED_MULTIPLIER;
        double joystickVY = -yVelSupplier.getAsDouble() * SPEED_MULTIPLIER;

        ChassisSpeeds speeds = drive.getState().Speeds;
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        Translation2d robotDisplacement = robotVelocity.times(timeOfFlight);

        Translation2d compensatedTarget = modifiedTarget.minus(robotDisplacement);

        double compensatedDistance = compensatedTarget.getDistance(currentPosition);

        ShooterStateData compensatedState = ShooterSubsystem.SHOOTER_MAP.get(compensatedDistance);

        double targetRPS = compensatedState.rps;
        Angle targetHoodAngle = compensatedState.hoodPos;

        SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(5, 0, 0)
                .withVelocityX(joystickVX)
                .withVelocityY(joystickVY)
                .withTargetDirection(calcDesiredHeading(currentPose, compensatedTarget))
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

        return Commands.run(() -> drive.setControl(request))
                .alongWith(hood.moveToPosition(targetHoodAngle))
                .alongWith(shooter.shoot(targetRPS));
    }
    ;
}
