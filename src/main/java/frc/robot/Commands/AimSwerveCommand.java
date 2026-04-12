package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LandMarks;
import frc.robot.Commands.GeneralRobotCommands.SwerveState;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.LEDSubsystem.Section;
import swervelib.SwerveInputStream;

public class AimSwerveCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier turnSupplier;
  private LEDSubsystem ledSubsystem;

  private static final double kMaxTurnScale = 1;

  private final PIDController controller = new PIDController(0.014, 0, 0);

  private Rotation2d desiredAngle;

  public AimSwerveCommand(
      SwerveSubsystem swerveSubsystem,
      LEDSubsystem ledSubsystem,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier turnSupplier,
      Rotation2d desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.leftYSupplier = leftYSupplier;
    this.leftXSupplier = leftXSupplier;
    this.turnSupplier = turnSupplier;
    this.ledSubsystem = ledSubsystem;
    this.desiredAngle = desiredAngle;
    controller.enableContinuousInput(-180, 180);

    addRequirements(swerveSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setState(SwerveState.AIMING);
    ledSubsystem.setBlink(Color.kGreen, Seconds.of(0.5), Section.SIDE);
  }

  @Override
  public void execute() {
    if (swerveSubsystem.isAimed()) {
      if (Math.abs(leftXSupplier.getAsDouble()) < OperatorConstants.OVERRIDE_DEADBAND
          && Math.abs(leftYSupplier.getAsDouble()) < OperatorConstants.OVERRIDE_DEADBAND) {
        swerveSubsystem.lockPose();
        swerveSubsystem.setState(SwerveState.LOCKED_AND_AIMED);
        ledSubsystem.setSolidColor(Color.kGreen, Section.SIDE);
        return;
      } else {
        swerveSubsystem.setState(SwerveState.AIMED);
      }
    } else {
      ledSubsystem.setBlink(Color.kGreen, Seconds.of(0.5), Section.SIDE);
    }

    swerveSubsystem.setState(SwerveState.AIMING);

    double forward = leftYSupplier.getAsDouble()
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double strafe = leftXSupplier.getAsDouble()
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double turn = 0.0;

    double maxOmega = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
    turn = MathUtil.clamp(
        controller.calculate(swerveSubsystem.getHeading().getDegrees(),
            desiredAngle.getDegrees()) * maxOmega,
        -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe), turn, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setState(SwerveState.OPERATED);
    ledSubsystem.setOff();
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring() || Math.abs(turnSupplier.getAsDouble()) > 0.5;
  }
}