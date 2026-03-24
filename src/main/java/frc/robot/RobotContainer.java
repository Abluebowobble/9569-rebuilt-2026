// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.OperatingSystemMXBean;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants.OperatorConstants;
import frc.SilverKnightsLib.DriverFeedback;
import frc.SilverKnightsLib.InputShaper;
import frc.SilverKnightsLib.TapHoldBinder;
import frc.robot.Commands.SubsystemsController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  // controllers
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
      OperatorConstants.DRIVER_1_CONTROLLER_PORT);
  private final CommandXboxController xboxController = new CommandXboxController(
      OperatorConstants.DRIVER_2_CONTROLLER_PORT);

  private final DoubleSupplier leftYSupplier = () -> ps5Controller.getLeftY() * -1;
  private final DoubleSupplier leftXSupplier = () -> ps5Controller.getLeftX() * -1;
  private final DoubleSupplier turnSupplier = () -> ps5Controller.getRightX() * -1;

  private InputShaper inputShaper = new InputShaper(leftXSupplier, leftYSupplier, turnSupplier);

  SubsystemsController subsystemsController = new SubsystemsController(inputShaper::getShapedYInput, inputShaper::getShapedXInput, inputShaper::getShapedTurnInput)

  private final SendableChooser<Command> autoChooser;

  private final GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerveSubsystem, shooterSubsystem,
      intakeSubsystem, hoodSubsystem, feederSubsystem, conveyorSubsystem, ledSubsystem, inputShaper::getShapedYInput,
      inputShaper::getShapedXInput, turnSupplier);

  public RobotContainer() {
    // config bindings
    configureBindings();

    // register path planner commands
    registerCommands();

    // set up auto choose
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerCommands() {
    NamedCommands.registerCommand("reverseFeeder", generalRobotCommands.reverseFeedCommand());
    NamedCommands.registerCommand("runFeeder", generalRobotCommands.feedCommand());
    NamedCommands.registerCommand("runShooter", generalRobotCommands.spinUpShooterCommand());
    NamedCommands.registerCommand("intakeUp", intakeSubsystem.returnPositionCommand());
    NamedCommands.registerCommand("intakeDown", intakeSubsystem.intakePositionCommand());
    NamedCommands.registerCommand("runIntakeRoller", intakeSubsystem.runRollerCommand());
    NamedCommands.registerCommand("reverseIntakeRoller", intakeSubsystem.reverseRollerCommand());
    NamedCommands.registerCommand("aim", generalRobotCommands.aimSwerveCommand());
    NamedCommands.registerCommand("passBackHoodPosition", hoodSubsystem.feedFromNeutralCommand());
    NamedCommands.registerCommand(
        "debugPrint",
        Commands.print("DEBUG COMMAND: PATH PLANNER IS RUNNING"));
  }

  private void configureBindings() {
    testBindings();
    // compBindings();
  }

  public void test() {
    SmartDashboard.putNumber("left x supplier", ps5Controller.getLeftX());
    SmartDashboard.putNumber("left Y supplier", ps5Controller.getLeftY());
    SmartDashboard.putNumber("left turn supplier", ps5Controller.getRightX());
  }

  public void testBindings() {
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
    // ps5Controller.L2().whileTrue(shooterSubsystem.runCommand(RPM.of(2900)));
    // ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());

    // // xbox
    // ps5Controller.square().onTrue(intakeSubsystem.returnPositionCommand());
    // ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    // ps5Controller.circle().whileTrue(intakeSubsystem.agitatePivotCommand());

    // hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() -> -ps5Controller.getLeftY()));
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.setCommand(() ->
    // -xboxController.getLeftY()));

    // //test
    // shooterSubsystsem.setDefaultCommand(shooterSubsystem.runCommand(5300));
    // feederSubsystem.setDefaultCommand(Commands.run(() -> {
    // feederSubsystem.setPercentageOutput(-ps5Controller.getLeftY());
    // }, feederSubsystem));

    // ps5Controller.circle().whileTrue(feederSubsystem.runCommand());
    // conveyorSubsystem.setDefaultCommand(
    // Commands.run(() ->
    // conveyorSubsystem.setPercentageOutput(-ps5Controller.getRightY()),
    // conveyorSubsystem));
    // ps5Controller.circle().whileTrue(feederSubsystem.runCommand());
    // conveyorSubsystem.setDefaultCommand(
    // Commands.run(() ->
    // conveyorSubsystem.setPercentageOutput(-ps5Controller.getRightY()),
    // conveyorSubsystem));
    // ps5Controller.circle().onTrue(generalRobotCommands.aimSwerveCommand());
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5500));
    // ps5Controller.cross().whileTrue(ledSubsystem.flashbangCommand());
    // ledSubsystem
    // .setDefaultCommand(Commands.run(() ->
    // ledSubsystem.setProgressMask(leftYSupplier, LEDSubsystem.Section.ALL),
    // ledSubsystem));
    // ps5Controller.L2().whileTrue(generalRobotCommands.runShooterCommand());

    // ps5Controller.circle().onTrue(generalRobotCommands.aimSwerveCommand());
    // ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    // ledSubsystem.setDefaultCommand(Commands.run(() -> ledSubsystem.setOff(),
    // ledSubsystem));
    // ps5Controller.circle().whileTrue(Commands.run(() -> swerveSubsystem.drive(new
    // ChassisSpeeds(0, 0, Math.PI)),
    // swerveSubsystem).handleInterrupt(() -> swerveSubsystem.drive(new
    // ChassisSpeeds(0, 0, 0))));
    // ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    // ledSubsystem.setDefaultCommand(Commands.run(() -> ledSubsystem.setOff(),
    // ledSubsystem));
    // ps5Controller.circle().whileTrue(Commands.run(() -> swerveSubsystem.drive(new
    // ChassisSpeeds(0, 0, Math.PI)),
    // swerveSubsystem).handleInterrupt(() -> swerveSubsystem.drive(new
    // ChassisSpeeds(0, 0, 0))));
  }

  public void johnnyBindings() {
    // default commands
    hoodSubsystem.setDefaultCommand(generalRobotCommands.prepareShooterCommand());

    // scoring
    ps5Controller.L2().toggleOnTrue(generalRobotCommands.spinUpShooterCommand());
    // ps5Controller.L2().whileTrue(Commands.waitSeconds(OperatorConstants.HOLD_DELAY.magnitude()).andThen(generalRobotCommands.aimAndShootCommand()));
    ps5Controller.L2().whileTrue(Commands.waitSeconds(OperatorConstants.HOLD_DELAY.magnitude())
        .andThen(Commands.print("I\"M RUNINIGNGIGIGNNGIGNIN")));

    // on true spin up shooter
    // while true auto aim, turn on shooter if not on. do not turn off shooter on
    // release
    // on true again turn off shooter
    // for auto aim, if within a certain tolerance start swerve lock
    // if linear velocity input is no longer 0 within a tolerance, accept
    // information
    // when its back to 0, set it backt o swerve lock
    // if press the reverse feeder button, reverse feeder while button is pressed,
    // no agitate
    // rotation overide too
    // if move, dont run agitate command, reverse feeder and conveyor
    // make intake go back to previous position (stowed or intake) after agitate

    ps5Controller.L2().whileTrue(generalRobotCommands.spinUpShooterCommand());
    ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());
    ps5Controller.R3().whileTrue(generalRobotCommands.aimSwerveCommand());

    // misc swerve commands
    ps5Controller.L3().toggleOnTrue(generalRobotCommands.swerveLockCommand()); // should i bind this with shooting? ask
                                                                               // johnny
    ps5Controller.L3().toggleOnTrue(generalRobotCommands.swerveLockCommand()); // should i bind this with shooting? ask
                                                                               // johnny
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    // passing
    ps5Controller.povUp().toggleOnTrue(
        hoodSubsystem.feedFromNeutralCommand().onlyWhile(() -> swerveSubsystem.currentPoseIsValidForScoring())); //

    // intake
    ps5Controller.L1().toggleOnTrue(generalRobotCommands.intakeCommand());
    ps5Controller.povLeft().toggleOnTrue(generalRobotCommands.reverseIntakeRollerCommand());
    ps5Controller.R1().whileTrue(generalRobotCommands.reverseFeedCommand());
    ps5Controller.square().onTrue(Commands.either(
        intakeSubsystem.returnPositionCommand(),
        intakeSubsystem.intakePositionCommand(),
        intakeSubsystem::isStowed));
        intakeSubsystem::isStowed));

    // gooner
    ps5Controller.povRight().whileTrue(ledSubsystem.flashbangCommand());
  }

  public void compBindings() {
    // default commands
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    hoodSubsystem.setDefaultCommand(generalRobotCommands.prepareShooterCommand());

    // ps5
    ps5Controller.L2().whileTrue(generalRobotCommands.spinUpShooterCommand());
    ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());
    ps5Controller.L3().toggleOnTrue(swerveSubsystem.swerveLockCommand(
        () -> Math.sqrt(Math.pow(leftXSupplier.getAsDouble(), 2) + Math.pow(leftYSupplier.getAsDouble(), 2))));
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    ps5Controller.touchpad().whileTrue(generalRobotCommands.aimSwerveCommand());
    ps5Controller.povUp().toggleOnTrue(hoodSubsystem.feedFromNeutralCommand());
    ps5Controller.povRight().toggleOnTrue(ledSubsystem.flashbangCommand());

    // xbox
    xboxController.a().onTrue(
        Commands.either(
            intakeSubsystem.returnPositionCommand(),
            intakeSubsystem.intakePositionCommand(),
            intakeSubsystem::isStowed));

    ps5Controller.L1().toggleOnTrue(intakeSubsystem.runRollerCommand());
    xboxController.leftBumper().whileTrue(intakeSubsystem.reverseRollerCommand());
  }

  public Command getAutonomousCommand() {
    // return swerveSubsystem.driveToPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.PI)));
    return autoChooser.getSelected();
    // return Commands.run(() -> swerveSubsystem.drive(new ChassisSpeeds(0, 0,
    // Math.PI)),
    // swerveSubsystem).withTimeout(2);
  }

  public Command shootAuton() {
    Command feed = conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand());
    Command shoot = Commands.deadline(
        Commands.waitSeconds(10),
        Commands.parallel(
            shooterSubsystem.runCommand(RPM.of(5400)),
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
    Command shoot = Commands.deadline(Commands.waitSeconds(10),
        Commands.parallel(shooterSubsystem.runCommand(RPM.of(5300)),
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
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(-0.592, -0.67, Math.PI, new Rotation2d(Math.PI)));
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
