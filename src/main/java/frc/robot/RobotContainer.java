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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
  HoodSubsystem hoodSubsystem = new HoodSubsystem();


  private final SendableChooser<Command> autoChooser;

  GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerve, shooterSubsystem, intakeSubsystem,
      feederSubsystem, conveyorSubsystem, leftYSupplier, leftXSupplier,
      driveFieldOrientedAnglularVelocity);

  private boolean isSwerveLocked = false;

  public RobotContainer() {

    // NamedCommands.registerCommand("reverse feeder",
    //     conveyorSubsystem.reverseCommand().alongWith(feederSubsystem.reverseCommand()));
    // NamedCommands.registerCommand("run feeder",
    //     conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));
    // NamedCommands.registerCommand("run shooter",
    //     shooterSubsystem.runCommand());
    // NamedCommands.registerCommand("intake up",
    //     intakeSubsystem.returnPositionCommand());
    // NamedCommands.registerCommand("intake down",
    //     intakeSubsystem.intakePositionCommand());
    // NamedCommands.registerCommand("run intake roller",
    //     intakeSubsystem.runRollerCommand());
    // NamedCommands.registerCommand("reverse intake roller",
    //     intakeSubsystem.reverseRollerCommand());
    // NamedCommands.registerCommand("agitate intake",
    //     intakeSubsystem.reverseRollerCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    
    configureBindings();
  }

  private void configureBindings() {
    testBindings();
    // compBindings();
    // compBindingsWithManualAgitate();
  }

  public void testBindings() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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

    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runVoltageCommand(() -> -ps5Controller.getLeftY()));
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

    // hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() -> -xboxController.getLeftY()));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.setCommand(() -> -xboxController.getLeftY()));
  }

  public void compBindings() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ps5
    ps5Controller.L2().whileTrue(shooterSubsystem.runCommand(4000));
    ps5Controller.R2().whileTrue(generalRobotCommands.feed());

    // xbox
    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
  }

  public void compBindingsWithManualAgitate() {
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ps5Controller.L2().whileTrue(shooterSubsystem.runCommand(4000));
    ps5Controller.R2().whileTrue(generalRobotCommands.feed());
    ps5Controller.R1().whileTrue(conveyorSubsystem.reverseCommand().alongWith(feederSubsystem.reverseCommand()));
    ps5Controller.povDown().onTrue(swerve.zeroGyroCommand());
    // ps5Controller.L3().onTrue(Commands.runOnce(() -> isSwerveLocked =
    // !isSwerveLocked));

    // if (isSwerveLocked) {
    // CommandScheduler commandScheduler = CommandScheduler.getInstance();
    // commandScheduler.schedule(swerve.swerveLockCommand().repeatedly());
    // }

    ps5Controller.L3().whileTrue(swerve.swerveLockCommand().repeatedly());

    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
    xboxController.leftBumper().whileTrue(intakeSubsystem.reverseRollerCommand());

    // //test
    shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5300));
    hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() -> -xboxController.getRightY()));
    // xboxController.y().onTrue(hoodSubsystem.setCommand(0.156));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5500));
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

      Command driveVelocity2 = swerve
          .driveWithSetpointGenerator(() -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.5, 0, 0, swerve.getHeading()));

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
      Command driveVelocity1_1 = swerve
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.592, -0.67, Math.PI, new Rotation2d(0)));
      Command driveVelocity1_4 = swerve
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(0.592, 0.67, Math.PI, new Rotation2d(0)));
      Command driveVelocity2_2 = swerve
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.592, -0.67, Math.PI, new Rotation2d(180)));
      Command driveVelocity2_3 = swerve
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
    return swerve.driveToPose(new Pose2d(Meter.of(20), Meter.of(0), new Rotation2d(0)));
  }

  public Command testDriveForwardAuton() {
    return swerve.driveToPose(new Pose2d(Meter.of(2.47), Meter.of(4), new Rotation2d(0)));
  }

  public Command testSpinAuton() {
    return swerve.driveToPose(new Pose2d(Meter.of(0), Meter.of(0), new Rotation2d(Degree.of(90))));
  }

}
