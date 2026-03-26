package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.BehaviourConstants;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.LEDSubsystem.Section;
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
    private final DoubleSupplier turnSupplier;

    public static enum ShooterState {
        SHOOTING, IDLE;
    }

    public static enum IntakeState {
        INTAKE, STOWED, AGITATING;
    }

    public static enum RollerState {
        RUNNING, REVERSE, STOP;
    }

    public static enum HoodState {
        AIMING, IDLE, PASSING;
    }

    public static enum ConveyorState {
        RUNNING, REVERSE, STOP;
    }

    public static enum FeederState {
        RUNNING, REVERSE, STOP;
    }

    public static enum SwerveState {
        AIMING, OPERATED, LOCKED, AIMED, LOCKED_AND_AIMED;
    }

    public boolean isFeeding = false;

    public static enum ScoreFeedState {
        FEEDING, REVERSING, NOT_SCORING;
    }

    public ScoreFeedState scoreFeedState;

    public GeneralRobotCommands(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem,
            ConveyorSubsystem conveyorSubsystem, LEDSubsystem ledSubsystem, DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier, DoubleSupplier turnSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.turnSupplier = turnSupplier;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;

        scoreFeedState = ScoreFeedState.NOT_SCORING;
    }

    public Command aimSwerveCommand() {
        return Commands.deadline(
                new AutoAimNoCorrectionCommand(swerveSubsystem, ledSubsystem, leftYSupplier, leftXSupplier,
                        turnSupplier),
                autoAimLightsCommand());
    }

    public Command prepareShooterCommand() {
        return Commands.deadline(
                new PrepareShooterCommand(shooterSubsystem, hoodSubsystem, swerveSubsystem),
                shooterLightsCommand());
    }

    public Command feedCommand() {
        return Commands.parallel(
                feederSubsystem.runCommand(),
                Commands.waitSeconds(BehaviourConstants.DELAY_BEFORE_AGITATE.magnitude())
                        .andThen(conveyorSubsystem.runCommand()
                                .alongWith(intakeSubsystem.agitatePivotCommand())))
                .onlyIf(() -> !intakeSubsystem.isStowed());
    }

    public Command feedWithOverrideCommand(BooleanSupplier reverseButton) {
        return Commands.parallel(
                feederSubsystem.runWithOverrideCommand(reverseButton),
                Commands.waitSeconds(BehaviourConstants.DELAY_BEFORE_AGITATE.magnitude())
                        .andThen(conveyorSubsystem.runWithOverrideCommand(reverseButton)
                                .alongWith(intakeSubsystem.agitateWithOverrideCommand(reverseButton))))
                .onlyIf(() -> !intakeSubsystem.isStowed());
    }

    public Command scoringCommand(BooleanSupplier reverseButton) {
        // return Commands.deadline(
        // aimSwerveCommand(),
        // Commands.waitUntil(() -> swerveSubsystem.isAimed() && isReadyToShoot())
        // .andThen(feedCommand()),
        // spinUpShooterCommand());
        return Commands.run(() -> {
            feederSubsystem.set(FeederSubsystem.Speed.REVERSE);
            conveyorSubsystem.set(ConveyorSubsystem.Speed.REVERSE);
        }).withTimeout(0.1).andThen(
                Commands.deadline(
                        new AutoAimNoCorrectionCommand(swerveSubsystem, ledSubsystem, leftYSupplier, leftXSupplier,
                                turnSupplier),
                        Commands.either(
                                shooterSubsystem.runCommand(RPM.of(5300)),
                                Commands.idle(),
                                () -> MathUtil.isNear(shooterSubsystem.getMinimumVelocity(),
                                        ShooterSubsystem.kStartingVelocity.magnitude(), 300))))
                .onlyIf(() -> !intakeSubsystem.isStowed());
    }

    // public Command managedFeedCommand(BooleanSupplier reverseHeld) {
    // return Commands.run(() -> {
    // if (reverseHeld.getAsBoolean()) {
    // feederSubsystem.set(FeederSubsystem.Speed.REVERSE);
    // conveyorSubsystem.set(ConveyorSubsystem.Speed.REVERSE);
    // } else if (canFeed()) {
    // feederSubsystem.set(FeederSubsystem.Speed.RUN);
    // conveyorSubsystem.set(ConveyorSubsystem.Speed.RUN);
    // } else {
    // feederSubsystem.set(FeederSubsystem.Speed.STOP);
    // conveyorSubsystem.set(ConveyorSubsystem.Speed.STOP);
    // }
    // }, feederSubsystem, conveyorSubsystem)
    // .finallyDo(() -> {
    // feederSubsystem.set(FeederSubsystem.Speed.STOP);
    // conveyorSubsystem.set(ConveyorSubsystem.Speed.STOP);
    // });
    // }

    public Command spinUpShooterCommand() {
        return Commands.parallel(
                shooterSubsystem.runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY),
                shooterLightsCommand());
    }

    public Command intakeCommand() {
        return intakeSubsystem.intakeCommand().andThen(
                Commands.defer(() -> {
                    // DOESNT WORK :(
                    ChassisSpeeds velocity = swerveSubsystem.getRobotVelocity();
                    LinearVelocity minSpeed = MetersPerSecond.of(0.01);
                    if (Math.abs(velocity.vxMetersPerSecond) > minSpeed.magnitude()
                            || Math.abs(velocity.vyMetersPerSecond) > minSpeed.magnitude()) {
                        return Commands.runOnce(() -> conveyorSubsystem.set(ConveyorSubsystem.Speed.RUN));
                    } else {
                        return Commands.runOnce(() -> conveyorSubsystem.set(ConveyorSubsystem.Speed.STOP));
                    }
                }, Set.of(conveyorSubsystem)));
    }

    // good enough
    public Command shooterLightsCommand() {
        return Commands.run(() -> {
            if (isReadyToShoot()) {
                ledSubsystem.setBlink(Color.kGreen, Seconds.of(0.25), Section.SIDE);
            } else {
                ledSubsystem.setSolidColor(Color.kOrange, Section.SIDE);
            }
        });
    }

    public Command feedFromNeutralCommand() {
        return Commands.parallel(hoodSubsystem.feedFromNeutralCommand(),
                neutralFeedLightsCommand());
    }

    private double remember = 0;

    public Command neutralFeedLightsCommand() {
        remember = 0;
        return Commands.run(() -> {
            if (!shooterSubsystem.isVelocityWithinTolerance() && shooterSubsystem.getState() == ShooterState.SHOOTING && remember == 0) {
                ledSubsystem.setSolidColor(Color.kOrange, Section.SIDE);
                return;
            } else {
                remember += 1;
            }

            if (hoodSubsystem.isPositionWithinTolerance()) {
                ledSubsystem.setSolidColor(Color.kDarkBlue, Section.SIDE);
            } else {
                ledSubsystem.setBlink(Color.kDarkBlue, Seconds.of(0.25), LEDSubsystem.Section.SHOOTER);
            }
        });
    }

    public Command runIntakeRollerCommand() {
        return Commands.parallel(
                Commands.run(() -> ledSubsystem.setSolidColor(Color.kPurple, Section.SIDE)),
                intakeSubsystem.runRollerCommand());
    }

    public Command reverseIntakeRollerCommand() {
        return Commands.parallel(
                intakeSubsystem.reverseRollerCommand(),
                Commands.run(() -> ledSubsystem.setSolidColor(Color.kDarkOrange, Section.SIDE)));
    }

    public Command autoAimLightsCommand() {
        return Commands.run(() -> {
            if (swerveSubsystem.isAimed()) {
                ledSubsystem.setSolidColor(Color.kGreen, Section.SIDE);
            } else {
                ledSubsystem.setBlink(Color.kGreen, Seconds.of(0.25), Section.SIDE);
            }
        });
    }

    public Command swerveLockCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> swerveSubsystem.setState(SwerveState.LOCKED)),
                Commands.runOnce(() -> ledSubsystem.setSolidColor(Color.kGreen, Section.SIDE)),
                swerveSubsystem.swerveLockCommand(
                        // motion vector
                        () -> Math.sqrt(
                                Math.pow(leftXSupplier.getAsDouble(), 2) + Math.pow(leftYSupplier.getAsDouble(), 2))))
                .handleInterrupt(() -> swerveSubsystem.setState(SwerveState.OPERATED));
    }

    public Command reverseFeedCommand() {
        return conveyorSubsystem.reverseCommand().alongWith(feederSubsystem.reverseCommand());
    }

    public Command scoringLightsCommand() {
        return Commands.run(() -> {
            if (isReadyToShoot()) {
                if (swerveSubsystem.isAimed()) {
                    ledSubsystem.setSolidColor(Color.kGreen, Section.SIDE);
                } else {
                    ledSubsystem.setBlink(Color.kGreen, Seconds.of(0.25), Section.SIDE);
                }
            } else {
                ledSubsystem.setSolidColor(Color.kOrange, Section.SIDE);
            }
        });
    }

    public boolean isReadyToShoot() {
        return shooterSubsystem.isVelocityWithinTolerance() && hoodSubsystem.isPositionWithinTolerance();
    }
}
