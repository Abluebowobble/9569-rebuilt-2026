// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LandMarks;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // Swerve drive object
  private final SwerveDrive swerveDrive;

  // PhotonVision class for full field localization
  private Vision vision;
  private final Field2d photonField2d = new Field2d();

  private final Field2d field = new Field2d();

  private boolean isLocked = false;
  boolean blueAlliance;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    // checks alliance
    blueAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue;

    // sets starting pose based on alliance
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(0),
        Meter.of(0)),
        Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(Meter.of(0),
            Meter.of(0)),
            Rotation2d.fromDegrees(180));

    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    // try to open json files to create swerve
    try {
      File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED, startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    zeroGyro();

    // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1);

    // resynchronize absolute encoders and motor encoders periodically when they are
    // not moving
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1);

    setupPathPlanner();

    // Module absolute positions

    // Robot pose
    Pose2d currentPose = swerveDrive.getPose();
    field.setRobotPose(currentPose);

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroCommand));
  }

  /** sets up path planner */
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * sets swervedrive to at a given velocity, parameters: ChassisSpeeds velocity
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * If the operator is in the Blue Alliance Station, this should be 0 degrees. If
   * the operator is in the Red Alliance Station, this should be 180 degrees.
   */
  public Rotation2d getOperatorForwardDirection() {
    Rotation2d fieldOrientedHeading = swerveDrive.getPose().getRotation();
    Optional<DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();

    if (currentAlliance.isPresent() && currentAlliance.get() == DriverStation.Alliance.Red) {
      return fieldOrientedHeading.rotateBy(new Rotation2d(Degrees.of(180)));
    }

    return fieldOrientedHeading;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void zeroGyro() {
    if (!blueAlliance) {
      swerveDrive.zeroGyro();

      swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), new Rotation2d(180)));
    } else {
      swerveDrive.zeroGyro();
    }
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public boolean isAimed() {
    return MathUtil.isNear(getTargetHeadingInOperatorPerspective().getDegrees(),
        getHeadingInOperaturPerspective().getDegrees(),
        SwerveConstants.AIM_TOLERANCE.magnitude(), -360, 360);
  }

  private Rotation2d getHeadingInOperaturPerspective() {
    final Rotation2d currentHeadingInBlueAlliancePerspective = swerveDrive.getPose().getRotation();
    final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective
        .minus(getOperatorForwardDirection());

    return currentHeadingInOperatorPerspective;
  }

  public Rotation2d getTargetHeadingInOperatorPerspective() {
    return getTargetHeadingInFieldFrame().minus(getOperatorForwardDirection());
  }

  public String aimSuggestion() {
    if (isAimed()) {
      return "Aimed!";
    }

    Rotation2d headingInOperatorPerspective = getHeadingInOperaturPerspective();
    Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();

    // implement
    return "";
  }

  public Rotation2d getTargetHeadingInFieldFrame() {
    final Translation2d hubPosition = LandMarks.hubPosition();
    final Translation2d robotPosition = swerveDrive.getPose().getTranslation();

    return hubPosition.minus(robotPosition).getAngle();
  }

  /** drives to a given pose2d */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  /** centers all 4 modules to heading 0 */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /** drives robot given joystick left x, left y, and right x values */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /** sets swerve to drive field oriented, updates as velocity updates */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException, org.json.simple.parser.ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  public void lockPose() {
    if (!isLocked) {
      swerveDrive.lockPose();
    }

    isLocked = !isLocked;
  }

  public Command swerveLockCommand() {
    return runOnce(() -> swerveDrive.lockPose());
  }

  public Command zeroGyroCommand() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // field2d
    Pose2d currentPose = swerveDrive.getPose();
    field.setRobotPose(currentPose);

    // telemetry for manual aim
    SmartDashboard.putBoolean("Is Aimed?", isAimed());
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds) {
    swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }
}
