// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.SilverKnightsLib.SwerveController;
import frc.SilverKnightsLib.SwerveController.Speeds;
import frc.robot.Constants;
import frc.robot.LandMarks;
import frc.robot.Commands.GeneralRobotCommands.SwerveState;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // Swerve drive object
  private final SwerveDrive swerveDrive;
  private final SwerveController swerveController = new SwerveController(SwerveConstants.translationController,
      SwerveConstants.rotationController, false, false, SwerveConstants.driveSlewRateLimit,
      SwerveConstants.driveJerkRateLimit, SwerveConstants.autonSlewRateLimit, SwerveConstants.autonJerkRateLimit);

  // PhotonVision class for full field localization
  private Vision vision;

  private final Field2d kField = new Field2d();

  public final boolean kIsBlueAlliance;
  private static final Distance kPoseEdgeMargin = Meters.of(0.3);

  public static final Angle kAimTolerance = Degrees.of(2);

  private SwerveState swerveState = SwerveState.OPERATED;

  private static StructPublisher<Pose3d> publisher;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    // checks alliance
    kIsBlueAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue;

    // sets starting pose based on alliance
    Pose2d startingPose = kIsBlueAlliance ? new Pose2d(new Translation2d(
        Meter.of(0),
        Meter.of(0)),
        new Rotation2d(0))
        : new Pose2d(new Translation2d(
            Meter.of(0),
            Meter.of(0)),
            Rotation2d.fromDegrees(180));

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    // try to open json files to create swerve
    try {
      File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED.magnitude(), startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    kField.setRobotPose(startingPose);
    SmartDashboard.putData("field swerve", kField);

    // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    // swerveDrive.setAngularVelocityCompensation(true,
    // true,
    // 0.1);

    // resynchronize absolute encoders and motor encoders periodically when they are
    // not moving
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1);

    setupPathPlanner();
    vision = new Vision();

    publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose3d.struct).publish();
  }

  public void setState(SwerveState swerveState) {
    this.swerveState = swerveState;
  }

  public SwerveState getState() {
    return swerveState;
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
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
              new PIDConstants(4.79645, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.3, 0.0)
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

  public boolean isAimed() {
    return Math.abs(
        getTargetHeadingInFieldFrame().minus(getHeading()).getDegrees()) < kAimTolerance.magnitude();
  }

  /**
   * sets swervedrive to at a given velocity, parameters: ChassisSpeeds velocity
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();

    if (!isBlueAlliance()) {
      swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), new Rotation2d(Math.PI)));
    }
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public Rotation2d getTargetHeadingInFieldFrame() {
    // final Translation2d hubPosition = LandMarks.hubPosition();
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
    swerveDrive.lockPose();
  }

  public Command swerveLockCommand(DoubleSupplier supplier) {
    return runOnce(() -> setState(SwerveState.LOCKED))
        .andThen(() -> swerveDrive.lockPose()).repeatedly()
        .until(() -> supplier.getAsDouble() > Constants.OperatorConstants.OVERRIDE_DEADBAND);
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro);
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

  public Vision getVision() {
    return vision;
  }

  public boolean currentPoseIsValidForScoring() {
    Pose2d pose = getPose();

    if (pose == null) {
      return false;
    }

    final Translation2d translation = pose.getTranslation();
    final double x = translation.getX();
    final double y = translation.getY();

    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      return false;
    }

    double poseEdgeMargin = kPoseEdgeMargin.magnitude();
    double sectionLength = LandMarks.kAllianceFieldLength.magnitude();

    boolean inYBounds = y >= -poseEdgeMargin
        && y <= LandMarks.kFieldWidth.magnitude() + poseEdgeMargin;

    if (!inYBounds) {
      return false;
    }

    if (isBlueAlliance()) {
      return x >= -poseEdgeMargin
          && x <= sectionLength + poseEdgeMargin;
    } else {
      return x >= LandMarks.kFieldLength.magnitude() - sectionLength - poseEdgeMargin
          && x <= LandMarks.kFieldLength.magnitude() + poseEdgeMargin;
    }
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
  }

  public double distanceToHub() {
    return swerveDrive.getPose().getTranslation().getDistance(LandMarks.hubPosition());
    // return vision.distanceToPoint(swerveDrive.getPose(),
    //     new Pose2d(new Translation2d(Inches.of(182.105), Inches.of(158.845)), new Rotation2d(0)));
  }

  public double distanceToAllianceHubCentre() {
    return swerveDrive.getPose().getTranslation().getDistance(new Translation2d(LandMarks.allianceHubCentreX(), Inches.of(swerveDrive.getPose().getY())));
    
    // vision.distanceToPoint(swerveDrive.getPose(),
    //     new Pose2d(new Translation2d(LandMarks.allianceHubCentreX(), Inches.of(swerveDrive.getPose().getY())), new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance FRom Blue HUb",
        vision.distanceToPoint(swerveDrive.getPose(), Vision.kAprilTagField.getTagPose(26).get().toPose2d()));
    SmartDashboard.putBoolean("is aimed?", isAimed());
    // SmartDashboard.putNumber("YAW FOR AUTO CORRECTION",
    // getTargetHeadingInFieldFrame()
    // .minus(getHeading())
    // .getDegrees());
    SmartDashboard.putNumber("gyro", getHeading().getDegrees());

    // SmartDashboard.putNumber(swerveDrive.getMaximumChassisAngularVelocity() + "",
    // swerveDrive.getMaximumChassisVelocity());
    // angular 6.283185307179586 rad/s
    // linear velocity : 5.05968 m/s
    // field2d
    Pose2d currentPose = swerveDrive.getPose();
    kField.setRobotPose(currentPose);
    SmartDashboard.putData("field swerve", kField);
    SmartDashboard.putString("Current Swerve State", toString());
    SmartDashboard.putNumber("Distance from alliance hub", distanceToAllianceHubCentre());

    vision.useBestPoseFieldRelativeTEST(this::addVisionMeasurement,
        swerveDrive.getRobotVelocity());

    publisher.accept(new Pose3d(getPose()));
  }

  @Override
  public String toString() {
    switch (swerveState) {
      case LOCKED:
        return "LOCKED";
      case AIMED:
      case LOCKED_AND_AIMED:
        return "AIM: READY";
      case AIMING:
        return "AIM: NOT READY";
      case OPERATED:
      default:
        return "OPERATED";
    }
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> curStdDevs) {
    swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, curStdDevs);
  }

  public Command swerveControllerDrive(Translation2d targetTranslation, Rotation2d targetRotation) {
    return run(() -> {
      Rotation2d currentRotation = getPose().getRotation();
      Speeds speeds = swerveController.calculate(swerveDrive::getPose,
          () -> new Pose2d(targetTranslation, targetRotation == null ? currentRotation : targetRotation));
      ChassisSpeeds finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vx(), speeds.vy(), speeds.vr(),
          currentRotation);
      drive(finalSpeeds);
    });
  }

  public double getDistanceError() {
    return swerveController.getDistanceError();
  }

  // might not work
  public Command swerveWaypointDrive(Translation2d targetTranslation, Rotation2d targetRotation,
      LinearVelocity xVelocity, LinearVelocity yVelocity) {
    return run(() -> {
      // store target position and rotation
      Rotation2d currentRotation = getPose().getRotation();
      Speeds speeds = swerveController.calculate(swerveDrive::getPose,
          () -> new Pose2d(targetTranslation, targetRotation == null ? currentRotation : targetRotation));
      ChassisSpeeds finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xVelocity == null ? 0 : xVelocity.magnitude(),
          yVelocity == null ? 0 : yVelocity.magnitude(),
          speeds.vr(),
          currentRotation);
      swerveDrive.drive(finalSpeeds);
    });
  }
}
