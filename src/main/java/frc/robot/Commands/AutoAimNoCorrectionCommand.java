package frc.robot.Commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilities.LandMarks;
import swervelib.SwerveInputStream;

public class AutoAimNoCorrectionCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier turnSupplier;

  private static final double kMaxTurnScale = 1;

  private final PIDController controller = new PIDController(0.014, 0, 0);

  public AutoAimNoCorrectionCommand(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier turnSupplier) {
    this.swerveSubsystem = swerveSubsystem;
    this.leftYSupplier = leftYSupplier;
    this.leftXSupplier = leftXSupplier;
    this.turnSupplier = turnSupplier;
    controller.enableContinuousInput(-180, 180);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    double forward = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double strafe = MathUtil.applyDeadband(leftXSupplier.getAsDouble(), OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double turn = 0.0;

    double error = swerveSubsystem.getTargetHeadingInFieldFrame()
        .minus(swerveSubsystem.getHeading())
        .getDegrees();

    if (!swerveSubsystem.isAimed()) {
      double maxOmega = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
      turn = MathUtil.clamp(
          -controller.calculate(swerveSubsystem.getHeading().getDegrees(),
              swerveSubsystem.getTargetHeadingInFieldFrame().getDegrees()) * maxOmega,
          -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);
    }

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe), turn, true, false);
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring() || Math.abs(turnSupplier.getAsDouble()) > 0.5;
  }
}