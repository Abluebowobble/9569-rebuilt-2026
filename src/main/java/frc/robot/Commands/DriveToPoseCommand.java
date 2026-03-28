// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meter;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.SilverKnightsLib.SwerveController;
import frc.SilverKnightsLib.SwerveController.Speeds;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.LandMarks;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCommand extends Command {

  private final SwerveController swerveController;
  private final SwerveSubsystem swerveSubsystem;
  private Pose2d targetPose;
  private Pose2d adjustedTargetPose;
  private final double exitRadius;

  /** Creates a new DriveToPoseCommand. */
  public DriveToPoseCommand(SwerveController swerveController, SwerveSubsystem swerveSubsystem, Pose2d targetPose, double exitRadius) {
    this.swerveController = swerveController;
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    this.exitRadius = exitRadius;

    addRequirements(swerveSubsystem);
  }

  private Pose2d transformForAlliance(Pose2d pose) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      double newX = LandMarks.kFieldLength.in(Meter) - pose.getX();
      double newY = LandMarks.kFieldWidth.in(Meter) - pose.getY();

      // rotate 180 degrees (flip direction)
      var newRotation = pose.getRotation().plus(Rotation2d.fromDegrees(180));

      return new Pose2d(newX, newY, newRotation);
    }

    return pose; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    adjustedTargetPose = transformForAlliance(targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Speeds speeds = swerveController.calculate(() -> swerveSubsystem.getPose(), () -> adjustedTargetPose);
    ChassisSpeeds finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vx(), speeds.vy(), speeds.vr(), swerveSubsystem.getPose().getRotation());
    swerveSubsystem.drive(finalSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveController.getDistanceError() < exitRadius;
  }
}
