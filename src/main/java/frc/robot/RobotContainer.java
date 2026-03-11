// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.reduxrobotics.canand.CanandEventLoop;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);
  private final CommandXboxController xboxController = new CommandXboxController(1);

  // subsystems
  private final SwerveSubsystem driveBase = new SwerveSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
      () -> ps5Controller.getLeftY() * -1,
      () -> ps5Controller.getLeftX() * -1)
      .withControllerRotationAxis(ps5Controller::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(ps5Controller::getRightX,
      ps5Controller::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(driveBase.getSwerveDrive(),
      () -> -ps5Controller.getLeftY(),
      () -> -ps5Controller.getLeftX())
      .withControllerRotationAxis(() -> ps5Controller.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          ps5Controller.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              ps5Controller.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  // different swerve commands
  // Command driveFieldOrientedDirectAngle =
  // driveBase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
  // Command driveRobotOrientedAngularVelocity =
  // driveBase.driveFieldOriented(driveRobotOriented);
  // Command driveSetpointGen = driveBase.driveWithSetpointGeneratorFieldRelative(
  // driveDirectAngle);
  Command driveFieldOrientedDirectAngleKeyboard = driveBase.driveFieldOriented(driveDirectAngleKeyboard);
  // Command driveFieldOrientedAnglularVelocityKeyboard =
  // driveBase.driveFieldOriented(driveAngularVelocityKeyboard);
  // Command driveSetpointGenKeyboard =
  // driveBase.driveWithSetpointGeneratorFieldRelative(
  // driveDirectAngleKeyboard);

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public RobotContainer() {
    CanandEventLoop.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    if (RobotBase.isSimulation()) {
      driveBase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      driveBase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    ps5Controller.triangle().onTrue(intakeSubsystem.intakeCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
