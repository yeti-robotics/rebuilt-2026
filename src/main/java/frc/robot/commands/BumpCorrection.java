package frc.robot.commands;

import static frc.robot.constants.Constants.currentMode;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstantsAlpha;
import frc.robot.subsystems.drive.TunerConstantsBeta;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Set;

public class BumpCorrection {
    private final CommandSwerveDrivetrain drive;
    private final double bumpY = 4;
    private final SwerveRequest.FieldCentric driveRequest = currentMode == Constants.Mode.ALPHA
            ? new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsAlpha.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstantsAlpha.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            : new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsBeta.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstantsBeta.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final Pose3d leftCameraPose;
    private final Pose3d frontCameraPose;
    private final Pose3d rightCameraPose;

    public BumpCorrection(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        leftCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.leftCamTrans);
        frontCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.frontCamTrans);
        rightCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.rightCamTrans);
    }

    public Command correction(double velocity) {
        return Commands.defer(
                () -> drive.runOnce(() -> drive.applyRequest(() -> driveRequest.withVelocityX(velocity))
                        .onlyIf(() -> drive.getState().Pose.getY() - bumpY > 1)),
                Set.of(drive));
    }
}
