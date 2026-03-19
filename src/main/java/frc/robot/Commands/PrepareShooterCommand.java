// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LandMarks;
import frc.robot.Commands.PrepareShooterCommand.Shot;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareShooterCommand extends Command {

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }
  }

  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
    // Reverse interpolates the value to find percentage distance between the two known values
      (startValue, endValue, q) -> InverseInterpolator.forDouble()
          .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
    // uses previously generated value to interpolate shooter velocity and hood position
      (startValue, endValue, t) -> new Shot(
          Interpolator.forDouble()
              .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
          Interpolator.forDouble()
              .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  // to tune 
  static {
    distanceToShotMap.put(Meters.of(1.649778334547691), new Shot(5300, 0.187));
    distanceToShotMap.put(Meters.of(0.5748198000485322), new Shot(5300, 0));
    distanceToShotMap.put(Meters.of(2.4006216556413467), new Shot(5300, 0.304));
    distanceToShotMap.put(Meters.of(1.8874167417587684), new Shot(5300, 0.226));
    distanceToShotMap.put(Meters.of(1.378834779227017), new Shot(5300, 0.148));
    distanceToShotMap.put(Meters.of(2.4854897119801356), new Shot(5300, 0.273));
  }

  private final ShooterSubsystem shooterSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final Supplier<Pose2d> robotPoseSupplier;

  /** Creates a new AimShotCommand. */
  public PrepareShooterCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
      Supplier<Pose2d> robotPoseSupplier) {
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.robotPoseSupplier = robotPoseSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, hoodSubsystem);
  }

  public boolean isReadyToShoot() {
    return shooterSubsystem.isVelocityWithinTolerance() && hoodSubsystem.isPositionWithinTolerance();
  }

  private Distance getDistanceToHub() {
    final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
    final Translation2d hubPosition = LandMarks.hubPosition();

    return Meters.of(robotPosition.getDistance(hubPosition));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // find distance
    final Distance distanceToHub = getDistanceToHub();

    // get appropriate rpm and hood position pair
    final Shot shot = distanceToShotMap.get(distanceToHub);

    // set subsystems with calculated values
    shooterSubsystem.set(shot.shooterRPM);
    hoodSubsystem.setPosition(shot.hoodPosition);

    // telemetry
    SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
