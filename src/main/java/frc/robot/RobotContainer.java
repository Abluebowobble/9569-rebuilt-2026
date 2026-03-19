// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.GeneralRobotCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
      OperatorConstants.DRIVER_1_CONTROLLER_PORT);
  private final CommandXboxController xboxController = new CommandXboxController(
      OperatorConstants.DRIVER_2_CONTROLLER_PORT);

  private DoubleSupplier leftYSupplier = () -> ps5Controller.getLeftY() * -1;
  private DoubleSupplier leftXSupplier = () -> ps5Controller.getLeftX() * -1;

  // subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      leftYSupplier,
      leftXSupplier)
      .withControllerRotationAxis(ps5Controller::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  private final Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  private final SendableChooser<Command> autoChooser;

  private final GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerveSubsystem, shooterSubsystem,
      intakeSubsystem,
      hoodSubsystem, feederSubsystem, conveyorSubsystem, leftYSupplier, leftXSupplier,
      driveFieldOrientedAnglularVelocity);

  public RobotContainer() {

    // NamedCommands.registerCommand("reverse feeder",
    // conveyorSubsystem.reverseCommand().alongWith(feederSubsystem.reverseCommand()));
    // NamedCommands.registerCommand("run feeder",
    // conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));
    // NamedCommands.registerCommand("run shooter",
    // shooterSubsystem.runCommand());
    // NamedCommands.registerCommand("intake up",
    // intakeSubsystem.returnPositionCommand());
    // NamedCommands.registerCommand("intake down",
    // intakeSubsystem.intakePositionCommand());
    // NamedCommands.registerCommand("run intake roller",
    // intakeSubsystem.runRollerCommand());
    // NamedCommands.registerCommand("reverse intake roller",
    // intakeSubsystem.reverseRollerCommand());
    // NamedCommands.registerCommand("agitate intake",
    // intakeSubsystem.reverseRollerCommand());

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
  }

  private void configureBindings() {
    testBindings();
    // compBindings();
    // compBindingsWithManualAgitate();
  }

  public void testBindings() {
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ps5Controller.triangle().whileTrue(new StartEndCommand(() ->
    // intakeSubsystem.set(Volts.of(6)), () -> intakeSubsystem.set(Volts.of(0))));
    ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    ps5Controller.circle().onTrue(intakeSubsystem.returnPositionCommand());
    ps5Controller.cross().whileTrue(intakeSubsystem.agitatePivotCommand());
    ps5Controller.square().whileTrue(generalRobotCommands.feed());
    // ps5Controller.square().whileTrue(intakeSubsystem.returnPositionCommand());

    // ps5Controller.circle().whileTrue(feederSubsystem.runCommand());
    // ps5Controller.square().whileTrue(conveyorSubsystem.runCommand());
    // ps5Controller.cross().whileTrue(conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));
    // ps5Controller.circle().whileTrue(feederSubsystem.reverseCommand());
    // ps5Controller.triangle().onTrue(shooterSubsystem.runCommand());
    // ps5Controller.circle().onTrue(shooterSubsystem.stopCommand());

    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runVoltageCommand(() ->
    // -ps5Controller.getLeftY()));
    // ps5Controller.square().whileTrue(new StartEndCommand(() ->
    // shooterSubsystem.set(Volts.of(0.8)), () ->
    // shooterSubsystem.set(Volts.of(0))));

    // // ps5
    // ps5Controller.L2().whileTrue(shooterSubsystem.runCommand());
    // ps5Controller.R2().whileTrue(generalRobotCommands.feed());

    // // xbox
    // ps5Controller.square().onTrue(intakeSubsystem.returnPositionCommand());
    // ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    // ps5Controller.square().whileTrue(intakeSubsystem.runRollerCommand());

    // hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() ->
    // -xboxController.getLeftY()));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.setCommand(() ->
    // -xboxController.getLeftY()));

    // //test
    shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5300));
    hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() -> -xboxController.getRightY()));
    // xboxController.y().onTrue(hoodSubsystem.setCommand(0.156));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5500));
  }

  public void compBindings() {
    // default commands
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    hoodSubsystem.setDefaultCommand(generalRobotCommands.prepareShooterCommand());

    // ps5
    ps5Controller.L2().whileTrue(shooterSubsystem.runCommand(5300));
    ps5Controller.R2().whileTrue(generalRobotCommands.feed());
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    ps5Controller.L3().toggleOnTrue(swerveSubsystem.swerveLockCommand().repeatedly());

    // xbox
    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
    xboxController.leftBumper().whileTrue(intakeSubsystem.reverseRollerCommand());
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("New Auto");
    return shootAuton();
  }

  public Command shootAuton() {
    Command feed = conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand());
    Command shoot = Commands.deadline(
        Commands.waitSeconds(10),
        Commands.parallel(
            shooterSubsystem.runCommand(4000),
            Commands.waitSeconds(3).andThen(feed)));

    try {

      Command driveVelocity2 = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.5, 0, 0, swerveSubsystem.getHeading()));

      return Commands.sequence(
          // Commands.deadline(Commands.waitSeconds(0.152), driveVelocity1),
          shoot,
          Commands.deadline(Commands.waitSeconds(2), driveVelocity2));
    } catch (Exception e) {
      DriverStation.reportError("VELOCITY DID NOT WORK", false);
      return shoot;
    }
  }

  public Command shootFeederShootAuton() {
    Command feed = conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand());
    Command shoot = Commands.deadline(Commands.waitSeconds(10), Commands.parallel(shooterSubsystem.runCommand(4000),
        Commands.waitUntil(() -> shooterSubsystem.isVelocityWithinTolerance())
            .andThen(feed)));
    try {
      Command driveVelocity1_1 = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.592, -0.67, Math.PI, new Rotation2d(0)));
      Command driveVelocity1_4 = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(0.592, 0.67, Math.PI, new Rotation2d(0)));
      Command driveVelocity2_2 = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.592, -0.67, Math.PI, new Rotation2d(180)));
      Command driveVelocity2_3 = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(0.592, 0.67, Math.PI, new Rotation2d(180)));

      return Commands.sequence(
          shoot,
          Commands.deadline(Commands.waitSeconds(2), driveVelocity1_1));
    } catch (Exception e) {
      DriverStation.reportError("VELOCITY DID NOT WORK", false);
      return shoot;
    }
  }

  public Command testDriveForwardRobotRelativeAuton() {
    return swerveSubsystem.driveToPose(new Pose2d(Meter.of(20), Meter.of(0), new Rotation2d(0)));
  }

  public Command testDriveForwardAuton() {
    return swerveSubsystem.driveToPose(new Pose2d(Meter.of(2.47), Meter.of(4), new Rotation2d(0)));
  }

  public Command testSpinAuton() {
    return swerveSubsystem.driveToPose(new Pose2d(Meter.of(0), Meter.of(0), new Rotation2d(Degree.of(90))));
  }

}
