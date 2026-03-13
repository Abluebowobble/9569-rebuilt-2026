// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LandMarks;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimStandingStillCommand extends Command {
  private static final double kPoseEdgeMarginMeters = 0.1;
  private final SwerveSubsystem swerve;
  private boolean poseWarningIssued = false;

  /** Creates a new AimAndDriveCommand. */
  public AimStandingStillCommand(SwerveSubsystem s) {
    this.swerve = s;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  private boolean currentPoseIsValid() {
    Pose2d pose = swerve.getPose();

    if (pose == null) {
      return false;
    }

    final Translation2d translation = pose.getTranslation();
    final double x = translation.getX();
    final double y = translation.getY();

    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      return false;
    }

    // add limit to the location on the field
    return x >= -kPoseEdgeMarginMeters
        && x <= LandMarks.fieldLength + kPoseEdgeMarginMeters
        && y >= -kPoseEdgeMarginMeters
        && y <= LandMarks.fieldWidth + kPoseEdgeMarginMeters;
  }

  private boolean ensurePoseValidWithWarning() {
    final boolean valid = currentPoseIsValid();
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

    swerve.driveToPose(
        new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), swerve.getTargetHeadingInFieldFrame()));
  }

  @Override
  public boolean isFinished() {
    return !swerve.isAimed();
  }
}
