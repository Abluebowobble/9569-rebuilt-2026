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
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
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

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(driveBase.getSwerveDrive(),
      () -> -ps5Controller.getLeftY(),
      () -> -ps5Controller.getLeftX())
      .withControllerRotationAxis(() -> ps5Controller.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math
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

  Command driveFieldOrientedAnglularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
  Command driveFieldOrientedDirectAngleKeyboard = driveBase.driveFieldOriented(driveDirectAngleKeyboard);

  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  FeederSubsystem FeederSubsystem = new FeederSubsystem();
  ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
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

    // ps5Controller.triangle().onTrue(intakeSubsystem.intakeCommand());
    ps5Controller.circle().onTrue(FeederSubsystem.runCommand());
    ps5Controller.square().onTrue(conveyorSubsystem.runCommand());
    ps5Controller.cross().onTrue(conveyorSubsystem.runCommand().alongWith(FeederSubsystem.runCommand()));
    ps5Controller.cross().onTrue(conveyorSubsystem.runCommand().alongWith(FeederSubsystem.runCommand()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
