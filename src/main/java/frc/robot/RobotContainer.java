// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degree;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
      OperatorConstants.DRIVER_1_CONTROLLER_PORT);
  private final CommandXboxController xboxController = new CommandXboxController(
      OperatorConstants.DRIVER_2_CONTROLLER_PORT);

  DoubleSupplier leftYSupplier = () -> ps5Controller.getLeftY() * -1;
  DoubleSupplier leftXSupplier = () -> ps5Controller.getLeftX() * -1;

  // subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> ps5Controller.getLeftY() * -1,
      () -> ps5Controller.getLeftX() * -1)
      .withControllerRotationAxis(ps5Controller::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  FeederSubsystem feederSubsystem = new FeederSubsystem();
  ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerve, shooterSubsystem, intakeSubsystem,
      feederSubsystem, conveyorSubsystem, leftYSupplier, leftXSupplier,
      driveFieldOrientedAnglularVelocity);

  public RobotContainer() {

    // NamedCommands.registerCommand("FeedCommand", GeneralRobotCommands.feed());

    configureBindings();
  }

  private void configureBindings() {
    // testBindings();
    // compBindings();
    compBindingsWithManualAgitate();
  }

  public void testBindings() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ps5Controller.triangle().whileTrue(new StartEndCommand(() ->
    // intakeSubsystem.set(Volts.of(6)), () -> intakeSubsystem.set(Volts.of(0))));
    // ps5Controller.circle().onTrue(intakeSubsystem.intakePositionCommand());
    // ps5Controller.cross().whileTrue(intakeSubsystem.agitatePivotCommand());
    // ps5Controller.square().whileTrue(intakeSubsystem.returnPositionCommand());

    // ps5Controller.circle().whileTrue(feederSubsystem.runCommand());
    // ps5Controller.square().whileTrue(conveyorSubsystem.runCommand());
    // ps5Controller.cross().whileTrue(conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));
    // ps5Controller.circle().whileTrue(feederSubsystem.reverseCommand());
    // ps5Controller.triangle().whileTrue(shooterSubsystem.runCommand());

    // ps5Controller.square().whileTrue(new StartEndCommand(() ->
    // shooterSubsystem.set(Volts.of(0.8)), () ->
    // shooterSubsystem.set(Volts.of(0))));

     // ps5
    ps5Controller.L2().whileTrue(shooterSubsystem.runCommand());
    ps5Controller.R2().whileTrue(generalRobotCommands.feed());

    // xbox
    ps5Controller.square().onTrue(intakeSubsystem.returnPositionCommand());
    ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    ps5Controller.square().whileTrue(intakeSubsystem.runRollerCommand());
  }

  public void compBindings() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ps5
    ps5Controller.L2().whileTrue(shooterSubsystem.runCommand());
    ps5Controller.R2().whileTrue(generalRobotCommands.feed());

    // xbox
    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
  }

  public void compBindingsWithManualAgitate() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    ps5Controller.L2().whileTrue(shooterSubsystem.runCommand());
    ps5Controller.R2().whileTrue(conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));

    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
    xboxController.rightTrigger().whileTrue(intakeSubsystem.agitatePivotCommand());
  }

  public Command getAutonomousCommand() {
    return shootAuton();
  }

  public Command shootAuton() {
    return shooterSubsystem.runCommand().alongWith(generalRobotCommands.feed());
  }

  public Command testDriveForwardRobotRelativeAuton() {
    return swerve.driveToPose(new Pose2d(Meter.of(20), Meter.of(0), new Rotation2d(0)));
  }

  public Command testDriveForwardAuton() {
    return swerve.driveToPose(new Pose2d(Meter.of(2.47), Meter.of(4), new Rotation2d(0)));
  }

  public Command testSpinAuton() {
    return swerve.driveToPose(new Pose2d(Meter.of(0), Meter.of(0), new Rotation2d(Degree.of(90))));
  }

}
