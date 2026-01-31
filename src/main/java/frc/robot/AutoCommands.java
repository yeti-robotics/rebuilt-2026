package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConfigs;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.linslide.LinSlideSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.PathPlannerUtils;

import java.util.Optional;

public class AutoCommands {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem hood;
    private final Hopper hopper;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LinSlideSubsystem linSlide;
    private final ShooterSubsystem shooter;

    public  AutoCommands(
            Climber climber,
            CommandSwerveDrivetrain drivetrain,
            HoodSubsystem hood,
            Hopper hopper,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            LinSlideSubsystem linsSlide,
            ShooterSubsystem shooter) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.hopper = hopper;
        this.indexer = indexer;
        this.intake = intake;
        this.linSlide = linsSlide;
        this.shooter = shooter;
    }

    public Command shoot() {
        return Commands.sequence(
                hopper.spinHopper(0),
                indexer.runMotors(0),
                shooter.shoot(0));
    }

    public Command intake() {
        return Commands.sequence(
                linSlide.moveToPosition(0.4, true),
                intake.rollIn());
    }

    public Command oneCycleNeutralTowerLeft () {
        Optional<PathPlannerPath> startNeutral = PathPlannerUtils.loadPathByName("start-neutral_L-left");
        Optional<PathPlannerPath> neutralShoot = PathPlannerUtils.loadPathByName("neutral_L-shoot-left");
        Optional<PathPlannerPath> shootTower = PathPlannerUtils.loadPathByName("shoot-tower-left");

        PathPlannerAuto auto;

        var cmd =
                startNeutral.isEmpty() || neutralShoot.isEmpty() || shootTower.isEmpty()
                ? Commands.none()
                        : Commands.sequence(
                                shoot(),
                        AutoBuilder.followPath(startNeutral.get()),
                        intake()
                );

        auto = new PathPlannerAuto(cmd);
        return auto;
    }
}



//Optional<PathPlannerPath> lineJ = PathPlannerUtils.loadPathByName("lineJ");
//Optional<PathPlannerPath> jToLollipop = PathPlannerUtils.loadPathByName("jToLoli");
//Optional<PathPlannerPath> lollipopToL = PathPlannerUtils.loadPathByName("loliToL");
//
//PathPlannerAuto auto;
//PathPlannerAuto lollipathJ =
//        new PathPlannerAuto((AutoBuilder.followPath(jToLollipop.get())));
//PathPlannerAuto lollipathL =
//        new PathPlannerAuto((AutoBuilder.followPath(lollipopToL.get())));
//
//var cmd =
//        lineJ.isEmpty() || jToLollipop.isEmpty()
//                ? Commands.none()
//                : Commands.sequence(
//                AutoBuilder.followPath(lineJ.get()),
//                reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT),
//                reefAlignPPOTF.reefAlign(),
//                coralManipulator.transitionTo(CoralManipulatorState.L4),
//                coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
//                coralManipulator
//                        .transitionTo(CoralManipulatorState.STOWED)
//                        .withTimeout(0.5),
//                AutoBuilder.followPath(jToLollipop.get())
//                        .until(coralManipulator.grabber::hasCoral),
//                coralManipulator
//                        .transitionTo(CoralManipulatorState.STOWED)
//                        .withTimeout(1),
//                AutoBuilder.followPath(lollipopToL.get()),
//                reefAlignPPOTF.reefAlign(),
//                coralManipulator.transitionTo(CoralManipulatorState.CLIMB_L4),
//                coralManipulator.transitionTo(CoralManipulatorState.SCORE_CLIMB_L4),
//                coralManipulator
//                        .transitionTo(CoralManipulatorState.STOWED)
//                        .withTimeout(0.5));
//auto = new PathPlannerAuto(cmd);
//        return auto;