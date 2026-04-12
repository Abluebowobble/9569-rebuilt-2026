// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.OperatingSystemMXBean;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WaypointConstants;
import frc.SilverKnightsLib.DriverFeedback;
import frc.SilverKnightsLib.InputShaper;
import frc.SilverKnightsLib.TapHoldBinder;
import frc.robot.Commands.Autons;
import frc.robot.Commands.GeneralRobotCommands;
import frc.robot.Commands.GeneralRobotCommands.IntakeState;
import frc.robot.Commands.GeneralRobotCommands.ScoreFeedState;
import frc.robot.Commands.GeneralRobotCommands.SwerveState;
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

  // subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem(shooterSubsystem::shouldFeed,
      swerveSubsystem::distanceToHub);
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(shooterSubsystem::isThereAnError);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  InputShaper inputShaper = new InputShaper(leftXSupplier, leftYSupplier, turnSupplier);
  private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      swerveSubsystem.getSwerveDrive(),
      inputShaper::getShapedYInput,
      inputShaper::getShapedXInput)
      // leftYSupplier,
      // leftXSupplier)
      .deadband(0.05)
      .withControllerRotationAxis(turnSupplier)
      .allianceRelativeControl(true);

  private final Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity)
      .alongWith(Commands.runOnce(() -> swerveSubsystem.setState(SwerveState.OPERATED)));

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final GeneralRobotCommands generalRobotCommands = new GeneralRobotCommands(swerveSubsystem, shooterSubsystem,
      intakeSubsystem, hoodSubsystem, feederSubsystem, conveyorSubsystem, ledSubsystem, inputShaper::getShapedYInput,
      inputShaper::getShapedXInput, turnSupplier);

  DriverFeedback driverFeedback;

  public RobotContainer() {
    // driver feedback
    driverFeedback = new DriverFeedback(ps5Controller, xboxController);

    // register path planner commands
    // registerCommands();

    // set up auto choose
    configureAutos();
    // config bindings
    configureBindings();
  }

  private void configureAutos() {
    autoChooser.setDefaultOption("Empty", Commands.none());
    // autoChooser.addOption("Test Forward",
    // Autons.testbackwardAuton(generalRobotCommands));
    autoChooser.addOption("Test Rotate", Autons.testRotateAuton(generalRobotCommands));
    autoChooser.addOption("Middle Depot Auto", Autons.testMiddleAuton(generalRobotCommands));
    autoChooser.addOption("Left Auto", Autons.testleftAuton(generalRobotCommands));
    autoChooser.addOption("Right Auto", Autons.testRightAuton(generalRobotCommands));
    autoChooser.addOption("Middle No Depot Auto", shootAuton());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public DriverFeedback getDriverFeedback() {
    return driverFeedback;
  }

  private void registerCommands() {
    NamedCommands.registerCommand("reverseFeeder", generalRobotCommands.reverseFeedCommand());
    NamedCommands.registerCommand("runFeeder", generalRobotCommands.feedCommand());
    NamedCommands.registerCommand("runShooter", generalRobotCommands.spinUpShooterCommand());
    NamedCommands.registerCommand("intakeUp", intakeSubsystem.returnPositionCommand());
    NamedCommands.registerCommand("intakeDown", intakeSubsystem.intakePositionCommand());
    NamedCommands.registerCommand("runIntakeRoller", intakeSubsystem.runRollerCommand());
    NamedCommands.registerCommand("reverseIntakeRoller", intakeSubsystem.reverseRollerCommand());
    // NamedCommands.registerCommand("aim",
    // generalRobotCommands.aimSwerveCommand());
    NamedCommands.registerCommand("passBackHoodPosition", hoodSubsystem.feedFromNeutralCommand());
    NamedCommands.registerCommand(
        "debugPrint",
        Commands.print("DEBUG COMMAND: PATH PLANNER IS RUNNING"));
  }

  private void configureBindings() {
    testBindings();
    // johnnyBindings();
  }

  public void test() {
    SmartDashboard.putNumber("left x supplier", ps5Controller.getLeftX());
    SmartDashboard.putNumber("left Y supplier", ps5Controller.getLeftY());
    SmartDashboard.putNumber("left turn supplier", ps5Controller.getRightX());
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
    // ps5Controller.L2().whileTrue(shooterSubsystem.runCommand(RPM.of(2900)));
    // ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());

    // // xbox
    // ps5Controller.square().onTrue(intakeSubsystem.returnPositionCommand());
    // ps5Controller.triangle().onTrue(intakeSubsystem.intakePositionCommand());
    // ps5Controller.circle().whileTrue(intakeSubsystem.agitatePivotCommand());

    SmartDashboard.putNumber("PICK HOOD POSITION", 0);
    hoodSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() -> SmartDashboard.getNumber("PICK HOOD POSITION", 0)));
    // shooterSubsystem.setDefaultCommand(hoodSubsystem.setCommand(() ->
    // -ps5Controller.getLeftY()));
    // ps5Controller.L2().toggleOnTrue(shooterSubsystem.runCommand(RPM.of(5300)));
    ps5Controller.circle().whileTrue(conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand()));

    SmartDashboard.putNumber("PICK RPM", 0);
    shooterSubsystem.setDefaultCommand(shooterSubsystem.runTestCommand(() -> SmartDashboard.getNumber("PICK RPM", 0)));
    ps5Controller.R1().whileTrue(conveyorSubsystem.runCommand());

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
    // ps5Controller.circle().onTrue(generalRobotCommands.aimSwerveCommand());
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runCommand(5500));
    // ps5Controller.cross().whileTrue(ledSubsystem.flashbangCommand());
    // ledSubsystem
    // .setDefaultCommand(Commands.run(() ->
    // ledSubsystem.setProgressMask(leftYSupplier, LEDSubsystem.Section.ALL),
    // ledSubsystem));
    // ps5Controller.L2().whileTrue(generalRobotCommands.runShooterCommand());

    // ps5Controller.circle().onTrue(generalRobotCommands.aimSwerveCommand());
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
    // ledSubsystem.setDefaultCommand(Commands.run(() -> ledSubsystem.setOff(),
    // ledSubsystem));
    // ps5Controller.circle().whileTrue(Commands.run(() -> swerveSubsystem.drive(new
    // ChassisSpeeds(0, 0, Math.PI)),
    // swerveSubsystem).handleInterrupt(() -> swerveSubsystem.drive(new
    // // ChassisSpeeds(0, 0, 0))));
    // ps5Controller.L1().toggleOnTrue(generalRobotCommands.intakeCommand());
    // ps5Controller.square().onTrue(intakeSubsystem.togglePositionCommand());
    // ps5Controller.R1().whileTrue(generalRobotCommands.reverseFeedCommand());
    // ps5Controller.povLeft().toggleOnTrue(generalRobotCommands.reverseIntakeRollerCommand());
    // ps5Controller.L1().toggleOnTrue(shooterSubsystem.runCommand(RPM.of(2900)))
    // ps5Controller.circle().whileTrue(generalRobotCommands.feedWithOverrideCommand(()
    // -> ps5Controller.L1().getAsBoolean()));
    // ps5Controller.L2().whileTrue(generalRobotCommands.scoringCommand(() ->
    // ps5Controller.R1().getAsBoolean()));
    // ps5Controller.square().onTrue(intakeSubsystem.togglePositionCommand());
    // ps5Controller.R2().whileTrue(feederSubsystem.runCommand());
    ps5Controller.L1().toggleOnTrue(generalRobotCommands.intakeCommand());

  }

  public void johnnyBindings() {

    // default commands
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // // scoring
    ps5Controller.L2().toggleOnTrue(Commands.defer(() -> {
      if (swerveSubsystem.currentPoseIsValidForScoring()) {
        return generalRobotCommands.prepareShooterForHubCommand();
      }
      return generalRobotCommands.prepareShooterForPassCommand();
    }, Set.of(shooterSubsystem, hoodSubsystem)));

    // ps5Controller.L2().whileTrue(
    // Commands.waitSeconds(OperatorConstants.HOLD_DELAY.magnitude())
    // .andThen(generalRobotCommands.scoringCommand(() ->
    // ps5Controller.R1().getAsBoolean())));

    ps5Controller.R2().whileTrue(feederSubsystem.runCommand().alongWith(conveyorSubsystem.runCommand()));
    ps5Controller.R3().whileTrue(Commands.defer(() -> {
      if (swerveSubsystem.currentPoseIsValidForScoring()) {
        return generalRobotCommands.aimSwerveToHubCommand();
      }
      return generalRobotCommands.aimSwerveToAlliance();
    }, Set.of(swerveSubsystem)));

    // // misc swerve commands
    // ps5Controller.circle().toggleOnTrue(generalRobotCommands.runIntakeRollerCommand());
    // ps5Controller.circle()
    // .onTrue(Commands.defer(() -> generalRobotCommands.driveToWithAngle(
    // Autons.allianceRelative(WaypointConstants.BLUE_1_SHOOT,
    // swerveSubsystem.isBlueAlliance()),
    // 0.05,
    // swerveSubsystem.isBlueAlliance() ? new Rotation2d(0) : new
    // Rotation2d(Math.PI)), Set.of(swerveSubsystem)));
    ps5Controller.R3().toggleOnTrue(swerveSubsystem.swerveLockCommand(() -> Math.sqrt(
        Math.pow(leftXSupplier.getAsDouble(), 2) + Math.pow(leftYSupplier.getAsDouble(), 2))));
    ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());

    // passing
    // ps5Controller.povUp().toggleOnTrue(
    // hoodSubsystem.feedFromNeutralCommand().onlyWhile(() ->
    // !swerveSubsystem.currentPoseIsValidForScoring())); //

    // intake
    ps5Controller.L1().toggleOnTrue(generalRobotCommands.intakeCommand());
    ps5Controller.square().onTrue(intakeSubsystem.togglePositionCommand());
    ps5Controller.cross().whileTrue(
        Commands.defer(() -> {
          return intakeSubsystem.agitatePivotCommand(intakeSubsystem.getIntakeState());
        },
            Set.of(intakeSubsystem)));
    ps5Controller.R1().whileTrue(generalRobotCommands.reverseFeedCommand());
    ps5Controller.povLeft()
        .whileTrue(generalRobotCommands.reverseIntakeRollerCommand().alongWith(conveyorSubsystem.reverseCommand()));

    // gooner
    // ps5Controller.povRight().whileTrue(Commands.run(() -> {
    // ps5Controller.setRumble(RumbleType.kBothRumble, 1);
    // }));
    // ps5Controller.povRight()
    // .onTrue(Commands.runOnce(() -> shooterSubsystem.isDisabledMiddle =
    // !shooterSubsystem.isDisabledMiddle));
    // ps5Controller.povUp()
    // .onTrue(Commands.runOnce(() -> shooterSubsystem.isDisabledLeft =
    // !shooterSubsystem.isDisabledLeft));
    // ps5Controller.circle()
    // .onTrue(Commands.runOnce(() -> shooterSubsystem.isDisabledRight =
    // !shooterSubsystem.isDisabledRight));
  }

  // public void compBindings() {
  // // default commands
  // swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  // hoodSubsystem.setDefaultCommand(generalRobotCommands.prepareShooterCommand());

  // // ps5
  // ps5Controller.L2().whileTrue(generalRobotCommands.spinUpShooterCommand());
  // ps5Controller.R2().whileTrue(generalRobotCommands.feedCommand());
  // ps5Controller.L3().toggleOnTrue(swerveSubsystem.swerveLockCommand(
  // () -> Math.sqrt(Math.pow(leftXSupplier.getAsDouble(), 2) +
  // Math.pow(leftYSupplier.getAsDouble(), 2)),
  // swerveSubsystem.getState()));
  // ps5Controller.povDown().onTrue(swerveSubsystem.zeroGyroCommand());
  // ps5Controller.touchpad().whileTrue(generalRobotCommands.aimSwerveCommand());
  // ps5Controller.povUp().toggleOnTrue(hoodSubsystem.feedFromNeutralCommand());
  // ps5Controller.povRight().toggleOnTrue(ledSubsystem.flashbangCommand());

  // // xbox
  // xboxController.a().onTrue(
  // Commands.either(
  // intakeSubsystem.returnPositionCommand(),
  // intakeSubsystem.intakePositionCommand(),
  // intakeSubsystem::isStowed));

  // ps5Controller.L1().toggleOnTrue(intakeSubsystem.runRollerCommand());
  // xboxController.leftBumper().whileTrue(intakeSubsystem.reverseRollerCommand());
  // }

  public Command getAutonomousCommand() {
    swerveSubsystem.zeroGyro();
    // return swerveSubsystem.driveToPose(new Pose2d(new Translation2d(0, 0), new
    // Rotation2d(Math.PI)));
    // return autoChooser.getSelected();
    return Autons.testleftAuton(generalRobotCommands);
    // return shootAuton();
    // return Commands.run(() -> swerveSubsystem.drive(new ChassisSpeeds(0, 0,
    // Math.PI)),
    // swerveSubsyste.withTimeout(2);
  }

  public Command shootAuton() {
    Command feed = conveyorSubsystem.runCommand().alongWith(feederSubsystem.runCommand());
    Command shoot = Commands.deadline(
        Commands.waitSeconds(10),
        shooterSubsystem.runCommand(RPM.of(5300)),
        Commands.waitSeconds(3).andThen(feed));

    try {
      Command driveVelocity = swerveSubsystem
          .driveWithSetpointGenerator(
              () -> ChassisSpeeds.fromRobotRelativeSpeeds(0.5, 0, 0, swerveSubsystem.getHeading()));

      return Commands.sequence(
          // Commands.deadline(Commands.waitSeconds(0.152), driveVelocity1),
          shoot,
          Commands.deadline(Commands.waitSeconds(2), driveVelocity));
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
