package frc.robot.Commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.GeneralRobotCommands.SwerveState;
import frc.robot.Subsystems.SwerveSubsystem;

public class AimSwerveCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier turnSupplier;

  private static final double kMaxTurnScale = 1;

  private final PIDController controller = new PIDController(0.014, 0, 0);

  public AimSwerveCommand(
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
  public void initialize() {
    swerveSubsystem.setState(SwerveState.AIMING);
  }

  @Override
  public void execute() {
    if (swerveSubsystem.isAimed()) {
      if (Math.abs(leftXSupplier.getAsDouble()) < OperatorConstants.OVERRIDE_DEADBAND
          && Math.abs(leftYSupplier.getAsDouble()) < OperatorConstants.OVERRIDE_DEADBAND) {
        swerveSubsystem.lockPose();
        swerveSubsystem.setState(SwerveState.LOCKED_AND_AIMED);
        return;
      }
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
            swerveSubsystem.getTargetHeadingInFieldFrame().getDegrees()) * maxOmega,
        -maxOmega * kMaxTurnScale, maxOmega * kMaxTurnScale);

    swerveSubsystem.getSwerveDrive().drive(new Translation2d(forward, strafe), turn, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setState(SwerveState.OPERATED);
  }

  @Override
  public boolean isFinished() {
    return !swerveSubsystem.currentPoseIsValidForScoring() || Math.abs(turnSupplier.getAsDouble()) > 0.5;
  }
}