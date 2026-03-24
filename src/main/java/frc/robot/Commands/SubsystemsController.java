package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.SilverKnightsLib.InputShaper;
import frc.robot.Constants.BehaviourConstants;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.Subsystems.LEDSubsystem.Section;
import edu.wpi.first.wpilibj.util.Color;

public class SubsystemsController {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    private final Command driveFieldOrientedAnglularVelocity;

    private SwerveState swerveState = SwerveState.OPERATED;
    private IntakeState intakeState = IntakeState.STOWED;
    private FeederState feederState = FeederState.STOP;
    private ConveyorState conveyorState = ConveyorState.STOP;
    private ShooterState shooterState = ShooterState.IDLE;
    private HoodState hoodState = HoodState.IDLE;

    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final DoubleSupplier turnSupplier;

    public static enum ShooterState {
        SPINNING, IDLE;
    }

    public static enum IntakeState {
        INTAKE, STOWED, AGITATING;
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
        OPERATED, LOCKED, AIMED;
    }

    public SubsystemsController(DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier, DoubleSupplier turnSupplier) {
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.turnSupplier = turnSupplier;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                swerveSubsystem.getSwerveDrive(),
                leftYSupplier,
                leftXSupplier)
                .withControllerRotationAxis(turnSupplier)
                .allianceRelativeControl(true);

        driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    }

    public void setDefaultCommands() {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    public Command swerveDefault() {
        switch (swerveState) {
            case OPERATED:
                return driveFieldOrientedAnglularVelocity;
            case LOCKED:
                return swerveLockCommand();
            case AIMED:
                return aimSwerveCommand();
            default:
                return driveFieldOrientedAnglularVelocity;
        }
    }

    public Command aimSwerveCommand() {
        return Commands.deadline(
                new AutoAimNoCorrectionCommand(swerveSubsystem, leftYSupplier, leftXSupplier, turnSupplier),
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
                                .alongWith(intakeSubsystem.agitatePivotCommand())));
    }

    public Command scoringCommand() {
        return Commands.deadline(
                aimSwerveCommand(),
                Commands.waitUntil(() -> swerveSubsystem.isAimed() && isReadyToShoot())
                        .andThen(feedCommand()),
                spinUpShooterCommand()).andThen(spinUpShooterCommand());
    }

    public Command spinUpShooterCommand() {
        return Commands.parallel(
                shooterSubsystem.runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY),
                shooterLightsCommand());
    }

    public Command intakeCommand() {
        return intakeSubsystem.intakePositionCommand().andThen(
                Commands.parallel(
                        runIntakeRollerCommand(),
                        conveyorSubsystem.runCommand().onlyWhile(() -> {
                            ChassisSpeeds velocity = swerveSubsystem.getRobotVelocity();
                            LinearVelocity minSpeed = MetersPerSecond.of(0.05);
                            return Math.abs(velocity.vxMetersPerSecond) > minSpeed.magnitude()
                                    || Math.abs(velocity.vyMetersPerSecond) > minSpeed.magnitude();
                        })));
    }

    // good enough
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

    public Command feedFromNeutralCommand() {
        return Commands.parallel(hoodSubsystem.feedFromNeutralCommand(),
                neutralFeedLightsCommand());
    }

    public Command neutralFeedLightsCommand() {
        return Commands.run(() -> {
            if (hoodSubsystem.isPositionWithinTolerance()) {
                ledSubsystem.setSolidColor(Color.kGreen, LEDSubsystem.Section.SHOOTER);
            } else {
                ledSubsystem.setProgressMask(hoodSubsystem::progress, LEDSubsystem.Section.SHOOTER);
            }
        });
    }

    public Command runIntakeRollerCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> ledSubsystem.setSolidColor(Color.kPurple, Section.SIDE)),
                intakeSubsystem.runRollerCommand());
    }

    public Command reverseIntakeRollerCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> ledSubsystem.setSolidColor(Color.kDarkOrange, Section.SIDE)),
                intakeSubsystem.reverseRollerCommand());
    }

    public Command autoAimLightsCommand() {
        return Commands.run(() -> {
            if (swerveSubsystem.isAimed()) {
                ledSubsystem.setSolidColor(Color.kGreen, LEDSubsystem.Section.SIDE);
            } else {
                ledSubsystem.setSolidColor(Color.kDarkOrange, LEDSubsystem.Section.SIDE);
            }
        });
    }

    public Command swerveLockCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> ledSubsystem.setSolidColor(Color.kRed, Section.SIDE)),
                swerveSubsystem.swerveLockCommand(
                        // motion vector
                        () -> Math.sqrt(
                                Math.pow(leftXSupplier.getAsDouble(), 2) + Math.pow(leftYSupplier.getAsDouble(), 2))));
    }

    public Command reverseFeedCommand() {
        return conveyorSubsystem.reverseCommand().alongWith(feederSubsystem.reverseCommand());
    }

    public boolean isReadyToShoot() {
        return shooterSubsystem.isVelocityWithinTolerance() && hoodSubsystem.isPositionWithinTolerance();
    }
}
