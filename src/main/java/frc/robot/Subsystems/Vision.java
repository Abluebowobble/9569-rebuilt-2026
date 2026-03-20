// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.LandMarks;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Vision extends SubsystemBase {

  // field
  public final AprilTagFieldLayout kAprilTagField = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltAndymark);

  private final Field2d field = new Field2d();

  // Photon related values
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

  private static final Transform3d kCamToRobot = new Transform3d(
      new Translation3d(Units.inchesToMeters(13.717), Units.inchesToMeters(0), Units.inchesToMeters(-25.353391)),
      new Rotation3d(0, Units.degreesToRadians(-63), 0));

  private static final Set<Integer> PRIORITY_TAGS = Set.of(
      8, 5, 9, 10, 4, 3, 11, 2, 18, 27, 19, 20, 26, 25, 21, 24);

  // Distance Thresholds
  private static final Distance kMaxDistTrusted = Meters.of(6); // 6m for priority tags
  private static final Distance kMaxDistNormal = Meters.of(3); // 3m for non-priority tags

  // confidence based on different states of vision
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> curStdDevs;
  private static final double kMaxAmbiguity = 0.7;

  /**
   * This object expects a function as an initial parameter. i.e. in yagsl:
   * swerve.addVisionMeasurement()
   */
  public Vision() {
    camera = new PhotonCamera("alice");
    photonPoseEstimator = new PhotonPoseEstimator(kAprilTagField, kCamToRobot);
    curStdDevs = kSingleTagStdDevs;
  }

  @Override
  public void periodic() {
    // make consumer use camera
    // useBestCameraResults();
    // useBestPoseFieldRelative();
    // useBestPoseFieldRelativeTEST();
    if (DriverStation.getAlliance().isPresent())
      SmartDashboard.putString("alliance",
          DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? "Blue" : "Red");

    SmartDashboard.putData("Localized Field", field);
  }

  /** newest */
  public void useBestPoseFieldRelativeTEST(EstimateConsumer estimateConsumer, ChassisSpeeds speeds) {
    // Get the most recent camera pipeline result
    PhotonPipelineResult result = camera.getLatestResult();

    // Exit early if no AprilTags are visible
    if (!result.hasTargets())
      return;

    // First try to get a multi-tag pose estimate, since that is usually more stable
    latestEstimatedPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);

    // If multi-tag fails, fall back to the lowest-ambiguity single-tag estimate
    if (latestEstimatedPose.isEmpty()) {
      latestEstimatedPose = photonPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    // Only continue if we actually got a pose estimate and there is at least one
    // detected target
    if (latestEstimatedPose.isPresent() && result.getTargets().size() > 0) {
      // Convert robot angular velocity from radians/sec to degrees/sec
      // Use absolute value so we only care about how fast the robot is spinning,
      // not which direction it is spinning
      double omegaDegrees = Math.abs(Units.radiansToDegrees(speeds.omegaRadiansPerSecond));

      // Only use this vision measurement if the robot is spinning faster than 50
      // deg/sec
      // Convert the estimated 3D pose into a 2D field-relative pose
      Pose2d pose2d = latestEstimatedPose.get().estimatedPose.toPose2d();

      // Show the raw estimated pose on the field widget for debugging
      field.setRobotPose(pose2d);

      // Recompute/update confidence values for this pose measurement
      updateEstimationStdDevs(latestEstimatedPose, result.getTargets(), omegaDegrees);

      // Store the 2D estimated pose again for readability
      Pose2d estimatedPose = latestEstimatedPose.get().estimatedPose.toPose2d();

      // Only pass the estimate into the consumer if:
      // 1. the standard deviations are valid
      // 2. the pose is still within the field boundaries
      if (curStdDevs.get(0, 0) != Double.MAX_VALUE && isPoseInField(estimatedPose)) {
        // Send the field-relative pose, timestamp, and confidence to the pose estimator
        // / consumer
        estimateConsumer.accept(
            estimatedPose,
            latestEstimatedPose.get().timestampSeconds,
            curStdDevs);

        // Update the field widget with the accepted pose
        field.setRobotPose(estimatedPose);
      }
    }
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public double distanceToBlueHub(Pose2d robotPose) {
    double distanceToTarget = PhotonUtils.getDistanceToPose(field.getRobotPose(), kAprilTagField.getTagPose(26).get().toPose2d());
    return distanceToTarget;
  }

  // /** Gets latest april tags stored in pipeline */
  // private void useBestPoseFieldRelative(EstimateConsumer estimateConsumer,
  // ChassisSpeeds speeds) {
  // List<PhotonTrackedTarget> targets = new ArrayList<>();

  // PhotonPipelineResult result = camera.getLatestResult();

  // if (result.hasTargets()) {
  // targets = result.getTargets();
  // latestEstimatedPose = photonPoseEstimator.update(result);

  // updateEstimationStdDevs(latestEstimatedPose, targets);
  // PhotonTrackedTarget bestTarget = result.getBestTarget();

  // // Calculate robot's field relative pose
  // if (kAprilTagField.getTagPose(bestTarget.getFiducialId()).isPresent()) {
  // Pose3d robotPose =
  // PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(),
  // kAprilTagField.getTagPose(bestTarget.getFiducialId()).get(), kCamToRobot);

  // Pose2d robotPose2d = robotPose.toPose2d();
  // field.setRobotPose(robotPose2d);
  // }

  // if (latestEstimatedPose.isPresent())
  // estimateConsumer.accept(latestEstimatedPose.get().estimatedPose.toPose2d(),
  // latestEstimatedPose.get().timestampSeconds,
  // curStdDevs);
  // }
  // }

  // /** Makes the given consumer use camera results for telemetry */
  // private void useBestCameraResults(EstimateConsumer estimateConsumer,
  // RobotState robotState) {
  // Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

  // for (var result : camera.getAllUnreadResults()) {

  // // process estimated pose using multitags, use lowest ambiguity pose as fall
  // // back
  // visionEstimate = photonPoseEstimator.estimateCoprocMultiTagPose(result);
  // if (visionEstimate.isEmpty()) {
  // visionEstimate = photonPoseEstimator.estimateLowestAmbiguityPose(result);
  // }

  // if (visionEstimate.isPresent()) {
  // // update confidence
  // updateEstimationStdDevs(visionEstimate, result.getTargets());

  // Pose2d estimatedPose = visionEstimate.get().estimatedPose.toPose2d();
  // if (curStdDevs.get(0, 0) != Double.MAX_VALUE && isPoseInField(estimatedPose))
  // {
  // consumer.accept(estimatedPose, visionEstimate.get().timestampSeconds,
  // curStdDevs);

  // field.setRobotPose(estimatedPose);
  // }
  // }
  // }
  // }

  private boolean isPoseInField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return x >= 0 && x <= LandMarks.fieldLength && y >= 0 && y <= LandMarks.fieldWidth;
  }

  /**
   * Updates our current confidence in how reliable the current vision measurement
   * is.
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, double omegaDegrees) {

    // If we failed to produce any pose estimate at all,
    // fall back to the default single-tag trust values.
    if (estimatedPose.isEmpty()) {
      curStdDevs = kSingleTagStdDevs;
      return;
    }

    // Variables used to score how trustworthy this frame is.
    double avgDist = 0;
    double avgAmbiguity = 0;
    boolean seesPriorityTag = false;
    int numTags = 0;

    // Go through each detected target and only use it if it exists
    // in the field layout.
    for (var tgt : targets) {
      var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

      // Ignore tags that are not in the official field layout.
      if (tagPose.isEmpty())
        continue;

      numTags++;

      // Measure how far the estimated robot pose is from this tag on the field.
      // Farther tags are generally less trustworthy.
      avgDist += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

      // Add this target's pose ambiguity.
      // Higher ambiguity means the camera is less certain about the solve.
      avgAmbiguity += tgt.poseAmbiguity;

      // Track whether we can see any priority tag.
      // Priority tags are trusted more later.
      if (PRIORITY_TAGS.contains(tgt.getFiducialId()))
        seesPriorityTag = true;
    }

    // If none of the detected targets were valid field tags,
    // fall back to default single-tag std devs.
    if (numTags == 0) {
      curStdDevs = kSingleTagStdDevs;
      return;
    }

    // Turn the accumulated sums into averages.
    avgDist /= numTags;
    avgAmbiguity /= numTags;

    // If a priority tag is visible, allow trusting single-tag measurements
    // from a bit farther away.
    double maxAllowedDist = seesPriorityTag ? kMaxDistTrusted.magnitude() : kMaxDistNormal.magnitude();

    // Reject this measurement entirely if the average ambiguity is too high.
    if (avgAmbiguity > kMaxAmbiguity) {
      curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      return;
    }

    // Reject this measurement entirely if we only see one tag and it is too far
    // away.
    if (numTags == 1 && avgDist > maxAllowedDist) {
      curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      return;
    }

    // Start with a base XY standard deviation:
    // multi-tag measurements are trusted more than single-tag measurements.
    double xyStdDev = (numTags > 1) ? 1.0 : 1.5;

    // Increase XY uncertainty as average distance increases.
    // The quadratic term makes far-away tags get penalized much more strongly.
    xyStdDev *= (1 + (Math.pow(avgDist, 2) / 30.0));

    // Clamp omega so that below 50 deg/s the rotation multiplier stays at 1.0.
    double clampedOmega = Math.max(50.0, omegaDegrees);

    // Reject vision entirely if the robot is spinning too fast,
    // since rotating quickly often causes poor AprilTag measurements.
    if (omegaDegrees > 240) {
      curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      return;
    }

    // Increase XY uncertainty as rotation speed increases.
    // Faster spinning -> trust translation from vision less.
    double rotationMultiplier = 1.0 + (clampedOmega - 50.0) * (7.0 / 190.0);
    xyStdDev *= rotationMultiplier;

    // If a priority tag is visible, reduce XY uncertainty
    // so the measurement is trusted more.
    if (seesPriorityTag) {
      xyStdDev *= 0.6;
    }

    // Final vision standard deviations:
    // X and Y use the dynamically computed uncertainty,
    // while theta is currently fixed.
    curStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 1);
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> curStdDevs);
  }
}
