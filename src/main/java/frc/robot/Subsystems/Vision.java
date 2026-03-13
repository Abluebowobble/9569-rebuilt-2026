// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Vision extends SubsystemBase {

  // field
  public static final AprilTagFieldLayout kAprilTagField = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltAndymark);

  // Photon related values
  private final PhotonCamera camera;
  private final PhotonPoseEstimator PhotonPoseEstimator;

  // darrien said: 1.709034 in y and 25.353391 in x
  private final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(Inches.of(0), Inches.of(1.709034), Inches.of(25.353391)),
      new Rotation3d(Degrees.of(0), Degrees.of(61.9), Degrees.of(0)));

  // swerve related values
  private final EstimateConsumer consumer;
  private Matrix<N3, N1> curStdDevs;

  // confidence based on different states of vision
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /**
   * This object expects a function as an initial parameter. i.e. in yagsl:
   * swerve.addVisionMeasurement()
   */
  public Vision(EstimateConsumer consumer) {
    this.consumer = consumer;
    camera = new PhotonCamera("camera");
    PhotonPoseEstimator = new PhotonPoseEstimator(kAprilTagField, kRobotToCam);
  }

  @Override
  public void periodic() {
    // make consumer use camera
    useBestCameraResults();
  }

  /** Gets latest april tags stored in pipeline */
  private List<PhotonTrackedTarget> updateTargets() {
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      targets = result.getTargets();
    }

    return targets;
  }

  /** Makes the given consumer use camera results for telemetry */
  private void useBestCameraResults() {
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

    for (var result : camera.getAllUnreadResults()) {

      // process estimated pose using multitags, use lowest ambiguity pose as fall
      // back
      visionEstimate = PhotonPoseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = PhotonPoseEstimator.estimateLowestAmbiguityPose(result);
      }

      // update confidence
      updateEstimationStdDevs(visionEstimate, result.getTargets());
    }

    // feed values to swerve
    consumer.accept(visionEstimate.get().estimatedPose.toPose2d(), visionEstimate.get().timestampSeconds, curStdDevs);
  }

  /** updates our current confidence in the reliability of the vision */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = PhotonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        if (tagPose.isEmpty())
          continue;

        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      // tune this shit
      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;

      } else {
        // One or more tags visible, continue
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
  
}
