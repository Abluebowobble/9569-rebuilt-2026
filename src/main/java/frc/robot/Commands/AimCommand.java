// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import java.util.function.DoubleSupplier;

// import com.pathplanner.lib.util.GeometryUtil;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Radians;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.LandMarks;
// import frc.robot.Subsystems.SwerveSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AimCommand extends Command {
//   private static final Angle kAimTolerance = Degrees.of(5);
//   private static final double kPoseEdgeMarginMeters = 0.1;
//   private final SwerveSubsystem swerve;
//   private boolean poseWarningIssued = false;
//   private double rotationalVelocity;
//   private DoubleSupplier leftSupplier;
//   private DoubleSupplier rightSupplier;

//   /** Creates a new AimAndDriveCommand. */
//   public AimCommand(SwerveSubsystem s, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
//     this.swerve = s;
//     this.leftSupplier = leftSupplier;
//     this.rightSupplier = rightSupplier;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(swerve);
//   }

//   public boolean isAimed() {
//     if (!currentPoseIsValid()) {
//       return false;
//     }
//     final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();
//     final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
//     final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective
//         .minus(swerve.getOperatorForwardDirection());
//     return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
//   }

//   private Rotation2d getTargetHeadingInOperatorPerspective() {
//     return getTargetHeadingInFieldFrame().minus(swerve.getOperatorForwardDirection());
//   }

//   private Rotation2d getTargetHeadingInFieldFrame() {
//     final Translation2d hubPosition = LandMarks.hubPosition();
//     final Translation2d robotPosition = swerve.getPose().getTranslation();

//     return hubPosition.minus(robotPosition).getAngle();
//   }

//   private boolean currentPoseIsValid() {
//     Pose2d pose = swerve.getPose();

//     if (pose == null) {
//       return false;
//     }

//     final Translation2d translation = pose.getTranslation();
//     final double x = translation.getX();
//     final double y = translation.getY();

//     if (!Double.isFinite(x) || !Double.isFinite(y)) {
//       return false;
//     }

//     return x >= -kPoseEdgeMarginMeters
//         && x <= LandMarks.fieldLength + kPoseEdgeMarginMeters
//         && y >= -kPoseEdgeMarginMeters
//         && y <= LandMarks.fieldWidth + kPoseEdgeMarginMeters;
//   }

//   public double rotationalVelocity() {
//     // find new rotational velocity
//     rotationalVelocity = swerve.getSwerveDrive().getSwerveController()
//         .headingCalculate(swerve.getHeading().getRadians(), getTargetHeadingInFieldFrame().getRadians());

//     // convert into controller readings
//     double velocityAsPercent = rotationalVelocity / swerve.getSwerveDrive().getMaximumChassisVelocity();

//     return velocityAsPercent;
//   }

//   private boolean isPoseValid(Pose2d pose) {
//     if (pose == null) {
//       return false;
//     }
//     final Translation2d translation = pose.getTranslation();
//     final double x = translation.getX();
//     final double y = translation.getY();
//     if (!Double.isFinite(x) || !Double.isFinite(y)) {
//       return false;
//     }
//     return x >= -kPoseEdgeMarginMeters
//         && x <= LandMarks.fieldLength + kPoseEdgeMarginMeters
//         && y >= -kPoseEdgeMarginMeters
//         && y <= LandMarks.fieldWidth + kPoseEdgeMarginMeters;
//   }

//   private boolean ensurePoseValidWithWarning() {
//     final boolean valid = currentPoseIsValid();
//     if (!valid) {
//       if (!poseWarningIssued) {
//         DriverStation.reportWarning("Auto aim blocked: robot pose outside field bounds", false);
//         poseWarningIssued = true;
//       }
//     } else {
//       poseWarningIssued = false;
//     }
//     return valid;
//   }

//   @Override
//   public void initialize() {
//     poseWarningIssued = false;
//     ensurePoseValidWithWarning();

//     // override default swerve command and run this
//     Command drive = swerve.driveCommand(leftSupplier, rightSupplier,
//         this::rotationalVelocity);

//     // schedule command
//     CommandScheduler.getInstance().schedule(drive);
//   }

//   @Override
//   public void execute() {
//     if (!ensurePoseValidWithWarning()) {
//       return;
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
