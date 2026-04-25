// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.constants.Constants.currentMode;
import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;
import static frc.robot.subsystems.feeder.FeederConfigsBeta.FEEDER_SPEED;
import static frc.robot.subsystems.indexer.IndexerConfigsBeta.TEST_INDEXER_SPEED;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.battery.BatteryFuelGauge;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.TunerConstantsAlpha;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOBeta;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOAlpha;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOAlpha;
import frc.robot.subsystems.intake.IntakeIOBeta;
import frc.robot.subsystems.linslide.LinSlide;
import frc.robot.subsystems.linslide.LinSlideConfigsBeta;
import frc.robot.subsystems.linslide.LinSlideIO;
import frc.robot.subsystems.linslide.LinSlideIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOGamma;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.sim.Mechanisms;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drive;
    private final LinSlide linSlide;
    private final Mechanisms mechanisms;
    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;
    private final Feeder feeder;
    private final Vision vision;
    private final Hood hood;
    private final AutoCommands autoCommands;
    private final BatteryFuelGauge battery;

    // Controller
    private final CommandXboxController controller =
            new CommandXboxController(Constants.PRIMARY_CONTROLLER_PORT); // real
    private final CommandXboxController controller2 = new CommandXboxController(Constants.GIGA_PORT); // testing

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric driveRequest = currentMode == Constants.Mode.ALPHA
            ? new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsAlpha.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstantsAlpha.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            : new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstantsBeta.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstantsBeta.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (currentMode) {
            case ALPHA:
                drive = TunerConstantsAlpha.createDrivetrain();
                linSlide = new LinSlide(new LinSlideIOReal());
                intake = new Intake(new IntakeIOAlpha());
                indexer = new Indexer(new IndexerIOAlpha());
                shooter = new Shooter(new ShooterIOReal());
                feeder = new Feeder(new FeederIOReal());
                hood = new Hood(new HoodIO() {});
                battery = new BatteryFuelGauge(0);
                vision = new Vision(
                        drive,
                        new VisionIOLimelight(
                                VisionConstants.frontCam,
                                drive.getRotation3d()::toRotation2d,
                                VisionConstants.frontLinearStdDevBaseline,
                                VisionConstants.frontAngularStdDevBaseline),
                        new VisionIOLimelight(
                                VisionConstants.leftCam,
                                drive.getRotation3d()::toRotation2d,
                                VisionConstants.leftLinearStdDevBaseline,
                                VisionConstants.leftAngularStdDevBaseline));
                break;

            case BETA:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = TunerConstantsBeta.createDrivetrain();
                linSlide = new LinSlide(new LinSlideIOReal());
                intake = new Intake(new IntakeIOBeta());
                indexer = new Indexer(new IndexerIOBeta());
                shooter = new Shooter(new ShooterIOGamma());
                feeder = new Feeder(new FeederIOReal());
                hood = new Hood(new HoodIOBeta());
                battery = new BatteryFuelGauge(0);
                vision = new Vision(
                        drive,
                        new VisionIOLimelight(
                                VisionConstants.frontCam,
                                drive.getRotation3d()::toRotation2d,
                                VisionConstants.frontLinearStdDevBaseline,
                                VisionConstants.frontAngularStdDevBaseline),
                        new VisionIOLimelight(
                                VisionConstants.leftCam,
                                drive.getRotation3d()::toRotation2d,
                                VisionConstants.leftLinearStdDevBaseline,
                                VisionConstants.leftAngularStdDevBaseline),
                        new VisionIOLimelight(
                                VisionConstants.rightCam,
                                drive.getRotation3d()::toRotation2d,
                                VisionConstants.rightLinearStdDevBaseLine,
                                VisionConstants.rightAngularStdDevBaseLine));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = TunerConstantsAlpha.createDrivetrain();
                linSlide = new LinSlide(new LinSlideIOReal());
                intake = new Intake(new IntakeIOAlpha());
                indexer = new Indexer(new IndexerIOAlpha());
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.frontCam, VisionConstants.frontCamTrans, () -> drive.getState().Pose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.leftCam, VisionConstants.leftCamTrans, () -> drive.getState().Pose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.rightCam, VisionConstants.rightCamTrans, () -> drive.getState().Pose));
                shooter = new Shooter(new ShooterIOReal());
                feeder = new Feeder(new FeederIOReal());
                hood = new Hood(new HoodIOBeta());
                battery = new BatteryFuelGauge(0);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = TunerConstantsAlpha.createDrivetrain();
                linSlide = new LinSlide(new LinSlideIO() {});
                intake = new Intake(new IntakeIO() {});
                indexer = new Indexer(new IndexerIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                feeder = new Feeder(new FeederIO() {});
                shooter = new Shooter((new ShooterIO() {}));
                hood = new Hood(new HoodIO() {});
                battery = new BatteryFuelGauge(0);

                break;
        }

        drive.setStateStdDevs(VecBuilder.fill(0.33333, 0.33333, Math.toRadians(0.5)));

        autoCommands = new AutoCommands(drive, hood, indexer, feeder, intake, linSlide, shooter);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up simulatable mechanisms
        mechanisms = new Mechanisms();

        autoChooser.addOption("Left", autoCommands.oneCycleNeutralTowerLeft());
        autoChooser.addOption("Right", autoCommands.twoCycleNeutralOutpostTowerRight());
        autoChooser.addOption("Cheesy Left", autoCommands.cheesyLeft());
        autoChooser.addOption("Cheesy Right", autoCommands.cheesyRight());
        autoChooser.addOption("DCMP L1", autoCommands.dcmpLeft());

        SmartDashboard.putNumber("Shooter Velocity", 0);

        // Configure the button bindings
        if (Robot.isReal()) {
            configureRealBindings();
            configureDebugBindings();
        } else if (Robot.isSimulation()) {
            configureSimBindings();
        }

        configureTriggers();
    }

    public void updateVisionSim() {
        Pose3d leftCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.leftCamTrans);
        Pose3d frontCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.frontCamTrans);
        Pose3d rightCameraPose = new Pose3d(drive.getState().Pose).transformBy(VisionConstants.rightCamTrans);
        Logger.recordOutput("Side Cam Transform", leftCameraPose);
        Logger.recordOutput("Front Cam Transform", frontCameraPose);
        Logger.recordOutput("Other Side Cam Transform", rightCameraPose);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureRealBindings() {
        drive.setDefaultCommand(drive.applyRequest(() -> driveRequest
                .withVelocityX(-controller.getLeftY() * TunerConstantsBeta.kSpeedAt12Volts.magnitude())
                .withVelocityY(-controller.getLeftX() * TunerConstantsBeta.kSpeedAt12Volts.magnitude())
                .withRotationalRate(-controller.getRightX() * TunerConstantsBeta.MaFxAngularRate)));

        controller.start().onTrue(Commands.runOnce(drive::seedFieldCentric, drive));

        controller.x().whileTrue(linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED));
        controller
                .b()
                .whileTrue(linSlide.applyPower(-LinSlideConfigsBeta.DEPLOY_SPEED)
                        .alongWith(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED)));
        controller
                .leftTrigger()
                .whileTrue(intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED)
                        .alongWith(linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED)
                                .until(linSlide::isDeployed)));

        controller
                .rightBumper()
                .onTrue(Commands.parallel(
                        intake.applyPower(-IntakeConfigsBeta.ROLLER_SPEED),
                        indexer.applyPower(-TEST_INDEXER_SPEED),
                        feeder.feed(-FEEDER_SPEED),
                        shooter.applyPower(-0.1),
                        linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED)));

        controller
                .leftBumper()
                .whileTrue(Commands.either(
                                AutoAimCommands.autoAim(
                                                drive,
                                                controller::getLeftY,
                                                controller::getLeftX,
                                                centerHubOpening.toTranslation2d())
                                        .alongWith(AutoAimCommands.readyAim(
                                                drive, shooter, hood, centerHubOpening.toTranslation2d())),
                                AutoAimCommands.shuttleAim(drive, controller::getLeftY, controller::getLeftX)
                                        .alongWith(AutoAimCommands.shuttleReadyAim(drive, shooter, hood)),
                                () -> AllianceFlipUtil.apply(
                                                drive.getState().Pose.getX())
                                        < 4.9)
                        .alongWith(linSlide.applyPower(LinSlideConfigsBeta.LINSLIDE_AUTO_SHOOT_SPEED)))
                .onFalse(hood.setHoodPosition(0));

        controller.povLeft().onTrue(hood.setHoodPosition(0));
        controller.povRight().onTrue(hood.setHoodPosition(0.65));

        controller
                .rightTrigger()
                .whileTrue(Commands.parallel(
                        indexer.applyPower(TEST_INDEXER_SPEED),
                        intake.applyPower(IntakeConfigsBeta.ROLLER_SPEED),
                        feeder.applyPower(1),
                        shooter.switchSlot(1)))
                .onFalse(shooter.switchSlot(0));
    }

    private void configureDebugBindings() {
        controller2.start().onTrue(Commands.runOnce(drive::seedFieldCentric, drive));

        //        controller2.leftBumper().whileTrue(intake.applyPower(IntakeConfigsBeta.ROLL_IN_SPEED));

        controller2
                .x()
                .whileTrue(linSlide.applyPower(LinSlideConfigsBeta.DEPLOY_SPEED))
                .onFalse(linSlide.applyPower(LinSlideConfigsBeta.STOP));
        controller2
                .b()
                .whileTrue(linSlide.applyPower(-LinSlideConfigsBeta.DEPLOY_SPEED))
                .onFalse(linSlide.applyPower(LinSlideConfigsBeta.STOP));

        controller2
                .leftTrigger()
                .whileTrue(AutoAimCommands.autoAim(
                                drive, controller2::getLeftY, controller2::getLeftX, centerHubOpening.toTranslation2d())
                        .alongWith(AutoAimCommands.readyAim(drive, shooter, hood, centerHubOpening.toTranslation2d())));

        controller2
                .rightTrigger()
                .whileTrue(indexer.applyPower(TEST_INDEXER_SPEED)
                        //                        .alongWith(intake.applyPower(IntakeConfigsBeta.ROLL_IN_SLOWER)
                        .alongWith(feeder.feed(FEEDER_SPEED)
                                .alongWith(linSlide.applyPower(LinSlideConfigsBeta.LINSLIDE_AUTO_SHOOT_SPEED))));
    }

    private void configureSimBindings() {
        drive.setDefaultCommand(drive.applyRequest(() -> driveRequest
                .withVelocityX(-controller.getLeftY() * TunerConstantsBeta.kSpeedAt12Volts.magnitude())
                .withVelocityY(-controller.getLeftX() * TunerConstantsBeta.kSpeedAt12Volts.magnitude())
                .withRotationalRate(-controller.getRightX() * TunerConstantsBeta.MaFxAngularRate)));

        controller.button(3).whileTrue(intake.rollIn());
        controller.button(4).whileTrue(intake.rollOut());

        controller
                .button(5)
                .onTrue(Commands.either(
                        linSlide.runIntake(-0.2, false), linSlide.runIntake(0.2, true), linSlide::isDeployed));

        controller
                .button(6)
                .whileTrue(Commands.either(
                        AutoAimCommands.autoAim(
                                        drive,
                                        controller::getLeftY,
                                        controller::getLeftX,
                                        centerHubOpening.toTranslation2d())
                                .alongWith(AutoAimCommands.readyAim(
                                        drive, shooter, hood, centerHubOpening.toTranslation2d())),
                        AutoAimCommands.shuttleAim(drive, controller::getLeftY, controller::getLeftX)
                                .alongWith(AutoAimCommands.shuttleReadyAim(drive, shooter, hood)),
                        () -> AllianceFlipUtil.apply(drive.getState().Pose.getX()) < 4.9));

        controller.button(7).whileTrue(indexer.spinIndexer(80));
        controller.button(8).onTrue(Commands.runOnce(drive::seedFieldCentric));
        controller
                .button(9)
                .whileTrue(AutoAimCommands.autoAimWithOrbit(
                        drive, controller::getLeftY, controller::getLeftX, centerHubOpening.toTranslation2d()));
    }

    public void updateMechanisms() {
        mechanisms.updateLinSlideMech(linSlide.getCurrentPosition());
    }

    public void configureTriggers() {
        // Undecided whether to use
    }

    public void updateLoggers() {
        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = AllianceFlipUtil.apply(centerHubOpening.toTranslation2d());
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        Logger.recordOutput("AutoAimCommands/Shooter Map/hub distance", distance);

        Translation2d shuttleTranslation = AllianceFlipUtil.apply(new Translation2d(2.35, currentPose.getY()));
        double shuttleDistance = shuttleTranslation.getDistance(currentPosition);

        Logger.recordOutput("AutoAimCommands/Shuttle Map/ideal shuttle distance", shuttleDistance);
    }

    public void saveLog() {
        battery.saveLog();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
