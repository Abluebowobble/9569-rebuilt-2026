package frc.robot.Commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LandMarks;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class AutoAimNoCorrectionCommand extends Command {
  private SwerveSubsystem swerve;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;

  private static final double kP = 0.014;
  private static final double kMaxTurnScale = 0.4;
  private boolean poseWarningIssued = false;

  public AutoAimNoCorrectionCommand(
      SwerveSubsystem swerve,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier) {
    this.swerve = swerve;
    this.leftYSupplier = leftYSupplier;
    this.leftXSupplier = leftXSupplier;

    addRequirements(swerve);
  }

  private boolean ensurePoseValidWithWarning() {
    final boolean valid = swerve.currentPoseIsValidForShooting();

    if (!valid) {
      if (!poseWarningIssued) {
        DriverStation.reportWarning("Auto aim blocked: robot pose outside field bounds", false);
        poseWarningIssued = true;
      }
    } else {
      poseWarningIssued = false;
    }

    return valid;
  }

  @Override
  public void initialize() {
    poseWarningIssued = false;
    ensurePoseValidWithWarning();
  }

  @Override
  public void execute() {
    double forward = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerve.getSwerveDrive().getMaximumChassisVelocity();
    double strafe = MathUtil.applyDeadband(leftXSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerve.getSwerveDrive().getMaximumChassisVelocity();
    double turn = 0;

    if (!ensurePoseValidWithWarning())
      return;

    // get change in yaw
    double yaw = swerve.getTargetHeadingInFieldFrame()
        .minus(swerve.getHeading())
        .getDegrees();

    if (!isAimed()) {
      double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
      turn = -yaw * kP * maxOmega;
      turn = MathUtil.clamp(turn, -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);
    } else {
      turn = 0.0;
    }

    swerve.getSwerveDrive().drive(new Translation2d(forward, strafe), turn, true, false);
    SmartDashboard.putBoolean("is aimed?", isAimed());
  }

  public boolean isAimed() {
    return Math.abs(
        swerve.getTargetHeadingInFieldFrame().minus(swerve.getHeading()).getDegrees()) < SwerveConstants.AIM_TOLERANCE
            .magnitude();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}