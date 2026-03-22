package frc.robot.Commands;

import java.lang.constant.Constable;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;
import frc.robot.Utilities.LandMarks;
import swervelib.SwerveInputStream;

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

  // @Override
  // public void initialize() {
  // Pose2d pose = Vision.kAprilTagField.getTagPose(26).get().toPose2d();
  // SwerveInputStream driveTestInputStream =
  // SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
  // leftYSupplier,
  // leftXSupplier)
  // // .aim(Vision.kAprilTagField.getTagPose(26).get().toPose2d())
  // .aim(pose).aimFeedforward(0.05, 0.1, 0)
  // .deadband(OperatorConstants.DEADBAND)
  // .allianceRelativeControl(true);
  // Command driveTestCommand =
  // swerveSubsystem.driveFieldOriented(driveTestInputStream);
  // swerveSubsystem.setDefaultCommand(driveTestCommand);
  // }

  @Override
  public void execute() {
    double forward = MathUtil.applyDeadband(leftYSupplier.getAsDouble(),
        OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity() * kMaxTranslationScale;
    double strafe = MathUtil.applyDeadband(leftXSupplier.getAsDouble(),
        OperatorConstants.DEADBAND)
        * swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity() * kMaxTranslationScale;
    double turn = 0.0;

    double error = swerveSubsystem.getTargetHeadingInFieldFrame()
        .minus(swerveSubsystem.getHeading())
        .getDegrees();

    double maxOmega = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();
    turn = MathUtil.clamp(
        controller.calculate(swerveSubsystem.getHeading().getDegrees(),
            swerveSubsystem.getTargetHeadingInFieldFrame().getDegrees()) * maxOmega,
        -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe),
        turn, true, false);
  }

  @Override
  public void end(boolean interrupted) {

    // SwerveInputStream driveAngularVelocity =
    // SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
    // leftYSupplier,
    // leftXSupplier)
    // .withControllerRotationAxis(turnSupplier)
    // .deadband(OperatorConstants.DEADBAND)
    // .cubeTranslationControllerAxis(true)
    // .cubeRotationControllerAxis(true)
    // .scaleTranslation(0.8)
    // .allianceRelativeControl(true);

    // Command driveFieldOrientedAnglularVelocity =
    // swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    // swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring()
        || Math.abs(turnSupplier.getAsDouble()) > Constants.OperatorConstants.OVERRIDE_DEADBAND;
  }
}