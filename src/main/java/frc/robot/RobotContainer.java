// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import com.reduxrobotics.canand.CanandEventLoop;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.GeneralRobotCommands;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(OperatorConstants.DRIVER_1_CONTROLLER_PORT);
  private final CommandXboxController xboxController = new CommandXboxController(OperatorConstants.DRIVER_2_CONTROLLER_PORT);

  DoubleSupplier leftXSupplier = () -> ps5Controller.getLeftX() * -1;
  DoubleSupplier leftYSupplier = () -> ps5Controller.getLeftY() * -1;

  // subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
      leftYSupplier,
      leftXSupplier)
      .withControllerRotationAxis(ps5Controller::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerve.getSwerveDrive(),
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

  Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
  Command driveFieldOrientedDirectAngleKeyboard = swerve.driveFieldOriented(driveDirectAngleKeyboard);

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  FeederSubsystem feederSubsystem = new FeederSubsystem();
  ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(vision, swerveSubsystem, shooterSubsystem,
            intakeSubsystem, hoodSubsystem, feederSubsystem,
            conveyorSubsystem, leftSupplier, rightSupplier,
            Command operatorSwerveDefaulCommand);

  public RobotContainer() {
    CanandEventLoop.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    if (RobotBase.isSimulation()) {
      swerve.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    // ps5Controller.triangle().onTrue(intakeSubsystem.intakeCommand());
    // ps5Controller.circle().onTrue(intakeSubsystem.returnCommand());
    // ps5Controller.cross().whileTrue(intakeSubsystem.agitateCommand());
    // ps5Controller.square().whileTrue(shooterSubsystem.runCommand(0.1));


    // ps5Controller.circle().whileTrue(feederSubsystem.runCommand());
    // ps5Controller.square().whileTrue(conveyorSubsystem.runCommand());
    // ps5Controller.cross().whileTrue(conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));
  }
  
  public void compBindings() {
    ps5Controller.L2().
  }
  public Command getAutonomousCommand() {
    return swerve.driveToPose(new Pose2d(Meter.of(0.2), Meter.of(0), new Rotation2d(0)));
  }
}
