// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.constants.FieldConstants.Hub.centerHubOpening;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOAlpha;
import frc.robot.subsystems.climber.ClimberPosition;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOAlpha;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOAlpha;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOAlpha;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.linslide.LinSlideIO;
import frc.robot.subsystems.linslide.LinSlideIOAlpha;
import frc.robot.subsystems.linslide.LinSlidePosition;
import frc.robot.subsystems.linslide.LinSlideSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOAlpha;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.util.sim.Mechanisms;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
    private final Drive drive;
    private final LED led;
    private final LinSlideSubsystem linSlide;
    private final Mechanisms mechanisms;
    private final IntakeSubsystem intake;
    private final Hopper hopper;
    private final Climber climber;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final Vision vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public void updateVisionSim() {
        //        Pose3d backCameraPose = new Pose3d(drive.get).transformBy(VisionConstants.backCamTrans);
        //
        //        Pose3d frontCameraPose = new Pose3d(drive.getPose()).transformBy(VisionConstants.frontCamTrans);

        //        Logger.recordOutput("Back Cam Transform", backCameraPose);
        //        Logger.recordOutput("Front Cam Transform", frontCameraPose);
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = TunerConstants.createDrivetrain();
                linSlide = new LinSlideSubsystem(new LinSlideIOAlpha());
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIOAlpha());
                hopper = new Hopper(new HopperIOAlpha());
                climber = new Climber(new ClimberIOAlpha());
                shooter = new ShooterSubsystem(new ShooterIOAlpha());
                indexer = new IndexerSubsystem(new IndexerIOAlpha());

                vision = null;
                break;

            case SIM:
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]));
                linSlide = new LinSlideSubsystem(new LinSlideIOAlpha());
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIOAlpha());
                hopper = new Hopper(new HopperIOAlpha());
                vision = null;
                climber = new Climber(new ClimberIOAlpha());
                shooter = new ShooterSubsystem(new ShooterIOAlpha());
                indexer = new IndexerSubsystem(new IndexerIOAlpha());

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = TunerConstants.createDrivetrain();
                linSlide = new LinSlideSubsystem(new LinSlideIO() {});
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIO() {});
                hopper = new Hopper(new HopperIO() {});
                climber = new Climber(new ClimberIO() {});
                vision = null;
                indexer = new IndexerSubsystem(new IndexerIO() {});
                shooter = new ShooterSubsystem((new ShooterIO() {}));

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up simulatable mechanisms
        mechanisms = new Mechanisms();

        // Set up SysId routines
        //        autoChooser.addOption("Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        //        autoChooser.addOption("Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        if (Robot.isReal()) {
            configureRealBindings();
        } else if (Robot.isSimulation()) {
            configureSimBindings();
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureRealBindings() {
        drive.setDefaultCommand(drive.applyRequest(() -> new SwerveRequest.FieldCentric()
                .withVelocityX(-controller.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude())
                .withVelocityY(-controller.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude())
                .withRotationalRate(-controller.getRightX() * TunerConstants.MaFxAngularRate)));
        controller.start().onTrue(Commands.runOnce(() -> drive.seedFieldCentric(), drive));

        controller.y().onTrue(climber.moveToPosition(ClimberPosition.L1.getHeight()));
        // controller.b().onTrue(climber.moveToPosition(ClimberPosition.BOTTOM.getHeight()));

        controller.rightBumper().whileTrue(intake.rollIn());
        controller.x().whileTrue(linSlide.applyPower(0.2)).onFalse(linSlide.applyPower(0));
        controller.b().whileTrue(linSlide.applyPower(-0.2)).onFalse(linSlide.applyPower(0));

        controller.leftBumper().whileTrue(intake.rollOut());
        //                .onTrue(Commands.either(
        //                        linSlide.moveToPosition(LinSlidePosition.STOW.getPosition()),
        //                        linSlide.moveToPosition(LinSlidePosition.DEPLOY.getPosition()),
        //                        linSlide::isDeployed));

        //        controller
        //                .leftTrigger()
        //                .whileTrue(AutoAimCommands.autoAim(
        //                                drive, controller::getLeftY, controller::getLeftX,
        // centerHubOpening.toTranslation2d())
        //                        .alongWith(shooter.shoot(100))
        //                        .alongWith(indexer.index(3)));

        controller.rightTrigger().whileTrue(hopper.spinHopper(80));
    }

    private void configureSimBindings() {
        //        drive.setDefaultCommand(
        //                drive.applyRequest(
        //                        () ->
        //                                drive.withVelocityX(
        //                                                -controller.getLeftY()
        //                                                        * TunerConstants.kSpeedAt12Volts
        //                                                        .magnitude())
        //                                        .withVelocityY(
        //                                                -controller.getLeftX()
        //                                                        * TunerConstants.kSpeedAt12Volts
        //                                                        .magnitude())
        //                                        .withRotationalRate(
        //                                                -controller.getRightX()
        //                                                        * TunerConstants.MaFxAngularRate)));
        //        controller
        //                .button(0)
        //                .onTrue(Commands.runOnce(
        //                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
        // Rotation2d.kZero)),
        //                                drive)
        //                        .ignoringDisable(true));

        controller.button(1).onTrue(climber.moveToPosition(ClimberPosition.L1.getHeight()));
        controller.button(2).onTrue(climber.moveToPosition(ClimberPosition.BOTTOM.getHeight()));

        controller.button(3).whileTrue(intake.rollIn());
        controller.button(4).whileTrue(intake.rollOut());

        controller
                .button(5)
                .onTrue(Commands.either(
                        linSlide.moveToPosition(LinSlidePosition.STOW.getPosition()),
                        linSlide.moveToPosition(LinSlidePosition.DEPLOY.getPosition()),
                        linSlide::isDeployed));

        //        controller
        //                .button(6)
        //                .whileTrue(AutoAimCommands.autoAim(
        //                                drive, controller::getLeftY, controller::getLeftX,
        // centerHubOpening.toTranslation2d())
        //                        .alongWith(shooter.shoot(100))
        //                        .alongWith(indexer.index(3)));

        controller.button(7).whileTrue(hopper.spinHopper(80));
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(climber.getCurrentPosition(), linSlide.getCurrentPosition(), true);
        mechanisms.publishComponentPoses(climber.getTargetPosition(), linSlide.getCurrentPosition(), false);
        mechanisms.updateClimberMechanism(climber.getCurrentPosition());
        mechanisms.updateLinSlideMech(linSlide.getCurrentPosition());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
