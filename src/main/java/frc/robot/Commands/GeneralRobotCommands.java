package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;

public class GeneralRobotCommands {

    SwerveSubsystem swerveSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    // HoodSubsystem hoodSubsystem;
    FeederSubsystem feederSubsystem;
    ConveyorSubsystem conveyorSubsystem;

    DoubleSupplier leftYSupplier;
    DoubleSupplier leftXSupplier;

    Command operatorSwerveDefaulCommand;

    public GeneralRobotCommands(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem,
            ConveyorSubsystem conveyorSubsystem, DoubleSupplier leftYSupplier, DoubleSupplier leftXSupplier,
            Command operatorSwerveDefaulCommand) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // this.hoodSubsystem = hoodSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;

        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;

        this.operatorSwerveDefaulCommand = operatorSwerveDefaulCommand;
    }

    // public Command AimAndShootCommand() {
    //     AimCommand aimCommand = new AimCommand(swerveSubsystem, leftYSupplier, leftXSupplier,
    //             operatorSwerveDefaulCommand);
    //     PrepareShooterCommand prepareShooterCommand = new PrepareShooterCommand(shooterSubsystem, hoodSubsystem,
    //             () -> swerveSubsystem.getPose());

    //     return Commands.deadline(
    //             Commands.waitUntil(() -> swerveSubsystem.isAimed()
    //                     && prepareShooterCommand.isReadyToShoot())
    //                     .andThen(feed()),
    //             aimCommand,
    //             Commands.waitSeconds(0.25).andThen(prepareShooterCommand));
    // }

    // public Command autoShooterNoAimCommand() {
    //     PrepareShooterCommand prepareShooterCommand = new PrepareShooterCommand(shooterSubsystem, hoodSubsystem,
    //             () -> swerveSubsystem.getPose());

    //     Command feedWhenReady = Commands.waitUntil(prepareShooterCommand::isReadyToShoot)
    //             .andThen(feed());

    //     return Commands.parallel(prepareShooterCommand, feedWhenReady);
    // }

    public Command feed() {
        return Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.parallel(
                        feederSubsystem.runCommand(),
                        Commands.waitSeconds(0.125)
                                .andThen(conveyorSubsystem.runCommand().alongWith(intakeSubsystem.agitatePivotCommand()))));
    }
}
