// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.GeneralRobotCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ConveyorSubsystem;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.HoodSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
      OperatorConstants.DRIVER_1_CONTROLLER_PORT);
  private final CommandXboxController xboxController = new CommandXboxController(
      OperatorConstants.DRIVER_2_CONTROLLER_PORT);

  private final DoubleSupplier leftYSupplier = () -> ps5Controller.getLeftY() * -1;
  private final DoubleSupplier leftXSupplier = () -> ps5Controller.getLeftX() * -1;
  private final DoubleSupplier turnSupplier = () -> ps5Controller.getRightX();
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
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final SendableChooser<Command> autoChooser;

  private final GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerveSubsystem, shooterSubsystem,
      intakeSubsystem, hoodSubsystem, feederSubsystem, conveyorSubsystem, ledSubsystem, leftYSupplier, leftXSupplier, turnSupplier);

  public RobotContainer() {

    // register path planner commands
    registerCommands();

    // set up auto choose
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // config bindings
    configureBindings();

    // warm up the path planner library
    FollowPathCommand.warmupCommand().schedule();
  }

  private void registerCommands() {
    NamedCommands.registerCommand("reverse feeder", generalRobotCommands.reverseFeedCommand());
    NamedCommands.registerCommand("run feeder", generalRobotCommands.feedCommand());
    NamedCommands.registerCommand("run shooter", generalRobotCommands.runShooterCommand());
    NamedCommands.registerCommand("intake up", intakeSubsystem.returnPositionCommand());
    NamedCommands.registerCommand("intake down", intakeSubsystem.intakePositionCommand());
    NamedCommands.registerCommand("run intake roller", intakeSubsystem.runRollerCommand());
    NamedCommands.registerCommand("reverse intake roller", intakeSubsystem.reverseRollerCommand());
    NamedCommands.registerCommand("aim", generalRobotCommands.aimSwerveCommand());
    NamedCommands.registerCommand("pass back hood position", hoodSubsystem.feedFromNeutralCommand());
  }

  private void configureBindings() {
    testBindings();
    // compBindings();
  }

  public void testBindings() {
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // // ps5Controller.triangle().whileTrue(new StartEndCommand(() ->
    // // intakeSubsystem.set(Volts.of(6)), () ->
    // intakeSubsystem.set(Volts.of(0))));
    // ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    // ps5Controller.circle().onTrue(intakeSubsystem.returnPositionCommand());
    // ps5Controller.cross().whileTrue(intakeSubsystem.agitatePivotCommand());
    // ps5Controller.square().whileTrue(generalRobotCommands.feedCommand());
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
    // ps5Controller.circle().whileTrue(intakeSubsystem.agitatePivotCommand());

    // hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() ->
    // -xboxController.getLeftY()));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.setCommand(() ->
    // -xboxController.getLeftY()));

    // //test
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5300));
    // hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() ->
    // -xboxController.getRightY()));
    ps5Controller.circle().onTrue(hoodSubsystem.setCommand(0.6));
    ps5Controller.square().onTrue(hoodSubsystem.setCommand(0));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5500));
    // ps5Controller.touchpad().whileTrue(ledSubsystem.flashBangCommand());
    // ledSubsystem
    //     .setDefaultCommand(Commands.run(() -> ledSubsystem.setProgressMask(leftYSupplier, LEDSubsystem.Section.ALL), ledSubsystem));
    //     ps5Controller.L2().whileTrue(generalRobotCommands.runShooterCommand());

    // ps5Controller.circle().toggleOnTrue(generalRobotCommands.aimSwerveCommand());
  }

  public void compBindings() {
    // default commands
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    hoodSubsystem.setDefaultCommand(generalRobotCommands.prepareShooterCommand());

    // ps5
    ps5Controller.L2().whileTrue(generalRobotCommands.runShooterCommand());
    ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    ps5Controller.touchpad().whileTrue(generalRobotCommands.aimSwerveCommand());
    ps5Controller.povUp().toggleOnTrue(hoodSubsystem.feedFromNeutralCommand());
    ps5Controller.povRight().toggleOnTrue(ledSubsystem.flashbangCommand());

    // xbox
    xboxController.x().onTrue(intakeSubsystem.returnPositionCommand());
    xboxController.a().onTrue(intakeSubsystem.intakePositionCommand());
    xboxController.leftTrigger().whileTrue(intakeSubsystem.runRollerCommand());
    xboxController.leftBumper().whileTrue(intakeSubsystem.reverseRollerCommand());
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("New Auto");
    return autoChooser.getSelected();
  }

  public Command shootAuton() {
    Command feed = conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand());
    Command shoot = Commands.deadline(
        Commands.waitSeconds(10),
        Commands.parallel(
            shooterSubsystem.runCommand(RPM.of(5300)),
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
    Command shoot = Commands.deadline(Commands.waitSeconds(10), Commands.parallel(shooterSubsystem.runCommand(RPM.of(5300)),
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
