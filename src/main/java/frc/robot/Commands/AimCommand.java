// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LandMarks;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
  private static final Angle kAimTolerance = Degrees.of(5);
  private final SwerveSubsystem swerve;
  private double rotationalVelocity;
  private DoubleSupplier leftSupplier;
  private DoubleSupplier rightSupplier;

  /** Creates a new AimAndDriveCommand. */
  public AimCommand(SwerveSubsystem s, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    this.swerve = s;
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public boolean isAimed() {
    return MathUtil.isNear(swerve.getHeading().getRadians(), getDirectionToHub().getRadians(),
        kAimTolerance.in(Radians), -Math.PI, Math.PI);
  }

  private Rotation2d getDirectionToHub() {
    Translation2d hubPosition = LandMarks.hubPosition();
    Translation2d robotPosition = swerve.getPose().getTranslation();

    // get vector and turn into angle
    Rotation2d hubDirectionInFieldPerspective = hubPosition.minus(robotPosition).getAngle();

    // // turn field oriented vector into robot oriented vector
    // Rotation2d hubDirectionInOperatorPerspective = hubDirectionInFieldPerspective
    // .minus(swerve.getHeading());

    return hubDirectionInFieldPerspective;
  }

  public double rotationalVelocity() {
    // find new rotational velocity
    rotationalVelocity = swerve.getSwerveDrive().getSwerveController()
        .headingCalculate(swerve.getHeading().getRadians(), getDirectionToHub().getRadians());

    // convert into controller readings
    double velocityAsPercent = rotationalVelocity / swerve.getSwerveDrive().getMaximumChassisVelocity();

    return velocityAsPercent;
  }

  @Override
  public void initialize() {

    // override default swerve command and run this
    Command drive = swerve.driveCommand(leftSupplier, rightSupplier,
        this::rotationalVelocity);

    // schedule command
    CommandScheduler.getInstance().schedule(drive);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
