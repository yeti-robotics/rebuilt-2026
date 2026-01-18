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
import frc.robot.commands.AutoAimCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOAlpha;
import frc.robot.subsystems.climber.ClimberPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConfigs;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOAlpha;
import frc.robot.subsystems.intake.IntakeConfigs;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOAlpha;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.linslide.LinSlideConfigsAlpha;
import frc.robot.subsystems.linslide.LinSlideIOAlpha;
import frc.robot.subsystems.linslide.LinSlideSubsystem;
import frc.robot.subsystems.linslide.LinearSlideIO;
import frc.robot.subsystems.shooter.ShooterConfigs;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOAlpha;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
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
    private final Drive drive;
    private final LED led;
    private final LinSlideSubsystem linSlide;
    private final Mechanisms mechanisms;
    private final IntakeSubsystem intake;
    private final Hopper hopper;
    private final Climber climber;
    private final ShooterSubsystem shooter;
    private final Vision vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public void updateVisionSim() {
        Pose3d backCameraPose = new Pose3d(drive.getPose()).transformBy(VisionConstants.backCamTrans);

        Pose3d frontCameraPose = new Pose3d(drive.getPose()).transformBy(VisionConstants.frontCamTrans);

        Logger.recordOutput("Back Cam Transform", backCameraPose);
        Logger.recordOutput("Front Cam Transform", frontCameraPose);
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                linSlide = new LinSlideSubsystem(new LinSlideIOAlpha());
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIOAlpha());
                hopper = new Hopper(new HopperIOAlpha());
                climber = new Climber(new ClimberIOAlpha());
                shooter = new ShooterSubsystem(new ShooterIOAlpha());

                vision = new Vision(
                        drive,
                        new VisionIOLimelight("Front Camera", drive::getRotation),
                        new VisionIOLimelight("Back Camera", drive::getRotation));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                linSlide = new LinSlideSubsystem(new LinSlideIOAlpha());
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIOAlpha());
                hopper = new Hopper(new HopperIOAlpha());
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim("Front Camera", VisionConstants.frontCamTrans, drive::getPose),
                        new VisionIOPhotonVisionSim("Back Camera", VisionConstants.backCamTrans, drive::getPose));
                climber = new Climber(new ClimberIOAlpha());
                shooter = new ShooterSubsystem(new ShooterIOAlpha());

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                linSlide = new LinSlideSubsystem(new LinearSlideIO() {});
                led = new LED();
                intake = new IntakeSubsystem(new IntakeIO() {});
                hopper = new Hopper(new HopperIO() {});
                climber = new Climber(new ClimberIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                shooter = new ShooterSubsystem((new ShooterIO() {}));

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up simulatable mechanisms
        mechanisms = new Mechanisms();

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Reset gyro to 0° when the start button is pressed
        controller
                .start()
                .onTrue(Commands.runOnce(
                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive)
                        .ignoringDisable(true));

        // Lock to 0° when D-Pad up button is held
        controller
                .povUp()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> Rotation2d.kZero));

        // Switch to X pattern when D-Pad down is pressed
        controller.povDown().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Climber
        controller.y().onTrue(climber.moveToPosition(ClimberPosition.L1.getHeight()));
        controller.a().onTrue(climber.moveToPosition(ClimberPosition.BOTTOM.getHeight()));

        // Intake
        controller.rightBumper().whileTrue(intake.setRoller(IntakeConfigs.INTAKE_ROLL_IN_VOLTAGE));
        controller.x().whileTrue(intake.setRoller(IntakeConfigs.INTAKE_ROLL_OUT_VOLTAGE));

        // Linear Slide - When isDeployed is true, it stows and when isDeployed is false, it deploys
        controller
                .leftBumper()
                .onTrue(Commands.either(
                        linSlide.moveToPosition(LinSlideConfigsAlpha.LINSLIDE_STOWED_POSITION),
                        linSlide.moveToPosition(LinSlideConfigsAlpha.LINSLIDE_DEPLOYED_POSITION),
                        linSlide::isDeployed));

        // Auto Align
        controller
                .leftTrigger()
                .whileTrue(AutoAimCommands.autoAim(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        centerHubOpening.toTranslation2d()));

        // Shooter + Hopper
        controller
                .rightTrigger()
                .whileTrue(shooter.shoot(ShooterConfigs.SHOOTING_VOLTAGE)
                        .alongWith(hopper.spinHopper(HopperConfigs.HOPPER_SPIN_VOLTAGE)));
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(climber.getCurrentPosition(), linSlide.getCurrentPosition(), true);
        mechanisms.publishComponentPoses(climber.getTargetPosition(), linSlide.getCurrentPosition(), false);
        mechanisms.updateClimberMechanism(climber.getCurrentPosition());
        mechanisms.updateElevatorMech(linSlide.getCurrentPosition());
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
