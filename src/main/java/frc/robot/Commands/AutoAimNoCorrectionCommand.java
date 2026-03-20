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
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;

  private static final double kP = 0.014;
  private static final double kMaxTurnScale = 0.4;

  public AutoAimNoCorrectionCommand(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier) {
    this.swerveSubsystem = swerveSubsystem;
    this.leftYSupplier = leftYSupplier;
    this.leftXSupplier = leftXSupplier;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    double forward = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double strafe = MathUtil.applyDeadband(leftXSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double turn = 0.0;

    double yaw = swerveSubsystem.getTargetHeadingInFieldFrame()
        .minus(swerveSubsystem.getHeading())
        .getDegrees();

    if (!swerveSubsystem.isAimed()) {
      double maxOmega = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
      turn = -yaw * kP * maxOmega;
      turn = MathUtil.clamp(turn, -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);
    }

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe), turn, true, false);
    SmartDashboard.putBoolean("is aimed?", swerveSubsystem.isAimed());
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring();
  }
}