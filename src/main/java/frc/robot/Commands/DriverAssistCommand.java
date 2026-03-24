// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.LandMarks;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriverAssistCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;

  public static final double kAssistScale = 0.1;
  private static final double kMaxDistance = 4.0;

  /** Creates a new DriverAssistCommand. */
  public DriverAssistCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  /** Returns the nearest ladder (either blue or red) */
  private Translation2d getClosestLadderTarget(Translation2d robotPosition) {
    Translation2d redTower = LandMarks.redTowerPosition();
    Translation2d blueTower = LandMarks.blueTowerPosition();

    if (robotPosition.getDistance(blueTower) < robotPosition.getDistance(redTower)) {
      return blueTower;
    } else {
      return redTower;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = swerveSubsystem.getPose();
    Translation2d robotPosition = pose.getTranslation();

    Translation2d ladderTarget = getClosestLadderTarget(robotPosition);

    if (robotPosition.getDistance(ladderTarget) > kMaxDistance) { // prevent command from activating when too far from ladder
      return;
    }

    Translation2d error = ladderTarget.minus(robotPosition);

    double maxVelocity = swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity();
    double maxAngularVelocity = swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity();

    double correctionX = error.getX() * maxVelocity * kAssistScale;

    boolean isTopHalf = robotPosition.getY() > ladderTarget.getY();

    Rotation2d desiredRotation;
    
    if (isTopHalf) {
      desiredRotation = Rotation2d.fromDegrees(-90);
    } else {
      desiredRotation = Rotation2d.fromDegrees(90);
    }

    double angleError = pose.getRotation().minus(desiredRotation).getRadians();

    double correctionOmega = -angleError * maxAngularVelocity * kAssistScale;

    ChassisSpeeds finalSpeeds = new ChassisSpeeds(correctionX, 0, correctionOmega);
    swerveSubsystem.drive(finalSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}