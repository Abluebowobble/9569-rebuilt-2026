package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoAimNoCorrectionCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier turnSupplier;

  private static final double kMaxTurnScale = 1;
  private static final double kMaxTranslationScale = 0.5;

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
    double forward = leftYSupplier.getAsDouble()
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity() * kMaxTranslationScale;
    double strafe = leftXSupplier.getAsDouble()
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity() * kMaxTranslationScale;
    double turn = 0.0;

    double maxOmega = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
    turn = MathUtil.clamp(
        controller.calculate(swerveSubsystem.getHeading().getDegrees(),
            swerveSubsystem.getTargetHeadingInFieldFrame().getDegrees()) * maxOmega,
        -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe),
        turn, true, false);
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring()
        || Math.abs(turnSupplier.getAsDouble()) > Constants.OperatorConstants.OVERRIDE_DEADBAND;
  }
}