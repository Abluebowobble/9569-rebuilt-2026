package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;
import edu.wpi.first.wpilibj.util.Color;

public class GeneralRobotCommands {

    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ConveyorSubsystem conveyorSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;

    public GeneralRobotCommands(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem,
            ConveyorSubsystem conveyorSubsystem, LEDSubsystem ledSubsystem, DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
    }

    // public Command AimAndShootCommand() {
    // AimCommand aimCommand = new AimCommand(swerveSubsystem, leftYSupplier,
    // leftXSupplier,
    // operatorSwerveDefaulCommand);
    // PrepareShooterCommand prepareShooterCommand = new
    // PrepareShooterCommand(shooterSubsystem, hoodSubsystem,
    // () -> swerveSubsystem.getPose());

    // return Commands.deadline(
    // Commands.waitUntil(() -> swerveSubsystem.isAimed()
    // && prepareShooterCommand.isReadyToShoot())
    // .andThen(feed()),
    // aimCommand,
    // Commands.waitSeconds(0.25).andThen(prepareShooterCommand));
    // }

    // public Command autoShooterNoAimCommand() {
    // PrepareShooterCommand prepareShooterCommand = new
    // PrepareShooterCommand(shooterSubsystem, hoodSubsystem,
    // () -> swerveSubsystem.getPose());

    // Command feedWhenReady =
    // Commands.waitUntil(prepareShooterCommand::isReadyToShoot)
    // .andThen(feed());

    // return Commands.parallel(prepareShooterCommand, feedWhenReady);
    // }

    public Command aimSwerveCommand() {
        return new AutoAimNoCorrectionCommand(swerveSubsystem, leftYSupplier, leftXSupplier);
    }

    public Command prepareShooterCommand() {
        return new PrepareShooterCommand(shooterSubsystem, hoodSubsystem, swerveSubsystem);
    }

    public Command feedCommand() {
        return Commands.parallel(
                feederSubsystem.runCommand(),
                Commands.waitSeconds(0.125)
                        .andThen(conveyorSubsystem.runCommand()
                                .alongWith(intakeSubsystem.agitatePivotCommand())));
    }

    // change to use isReadToShoot adw
    public Command runShooterCommand() {
        return Commands.parallel(
                shooterSubsystem.runCommand(5300),
                shooterLightsCommand());
    }

    public Command shooterLightsCommand() {
        return Commands
                .run(() -> {
                    if (isReadyToShoot()) {
                        ledSubsystem.setSolidColor(Color.kGreen, LEDSubsystem.Section.SHOOTER);
                    } else {
                        ledSubsystem.setProgressMask(shooterSubsystem::progress, LEDSubsystem.Section.SHOOTER);
                    }
                });
    }
// auto aim lights
    // public Command 

    public boolean isReadyToShoot() {
        return shooterSubsystem.isVelocityWithinTolerance() && hoodSubsystem.isPositionWithinTolerance();
    }
}
