// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

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

  // Cameras
  private final PhotonCamera cameraR;
  private final PhotonCamera cameraL;

  private final Transform3d kRobotToCamR = new Transform3d(new Translation3d(-0.13172, 0.32659, 0.338),
      new Rotation3d(0, 0, 0));
  private final Transform3d kRobotToCamL = new Transform3d(new Translation3d(-0.13172, -0.32659, 0.338),
      new Rotation3d(0, 0, 0));

  private final PhotonPoseEstimator photonEstimatorR;
  private final PhotonPoseEstimator photonEstimatorL;

  private Matrix<N3, N1> curStdDevsR;
  private Matrix<N3, N1> curStdDevsL;

  private Supplier<Pose2d> currentPose;

  private final EstimateConsumer consumer;

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /** Creates a new Vision. */
  public Vision(EstimateConsumer consumer) {
    this.consumer = consumer;

    cameraR = new PhotonCamera("right camera");
    cameraL = new PhotonCamera("left camera");

    photonEstimatorR = new PhotonPoseEstimator(kAprilTagField, kRobotToCamR);
    photonEstimatorL = new PhotonPoseEstimator(kAprilTagField, kRobotToCamL);
  }

  // public void updatePoseCamera(PhotonCamera camera, PhotonPoseEstimator
  // poseEstimator, Transform3d cameraToRobot) {
  // var result = camera.getLatestResult();

  // if (!result.hasTargets())
  // return;

  // PhotonTrackedTarget target = result.getBestTarget();

  // if (kAprilTagField.getTagPose(target.getFiducialId()).isPresent()) {
  // Pose3d robotPose =
  // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
  // kAprilTagField.getTagPose(target.getFiducialId()).get(), cameraToRobot);
  // }
  // Optional<EstimatedRobotPose> pose =
  // poseEstimator.estimateCoprocMultiTagPose(result);

  // if (pose.isEmpty()) {
  // pose = poseEstimator.estimateLowestAmbiguityPose(result);
  // }
  // }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run

    List<PhotonTrackedTarget> targetsR = updateResults(cameraR);
    List<PhotonTrackedTarget> targetsL = updateResults(cameraL);
  }

  private List<PhotonTrackedTarget> updateResults(PhotonCamera camera) {
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      targets = result.getTargets();
    }

    return targets;
  }

  private Optional<EstimatedRobotPose> processCamera(PhotonCamera camera, PhotonPoseEstimator photonEstimator, Matrix<N3, N1> curStdDevs) {
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      estimatedPose = photonEstimator.estimateCoprocMultiTagPose(result);
      if (estimatedPose.isEmpty()) {
        estimatedPose = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      
      updateEstimationStdDevs(estimatedPose, result.getTargets(), photonEstimator, curStdDevs);
    }
    return estimatedPose;
  }

  private void useBestCameraResults() {
    Optional<EstimatedRobotPose> visionEstL = processCamera(cameraL, photonEstimatorL, curStdDevsL);
    Optional<EstimatedRobotPose> visionEstR = processCamera(cameraR, photonEstimatorR, curStdDevsR);

    double sumR = curStdDevsR.elementSum();
    double sumL = curStdDevsL.elementSum();

    if (sumR < sumL && visionEstR.isPresent()) {
      consumer.accept(visionEstR.get().estimatedPose.toPose2d(), visionEstR.get().timestampSeconds, curStdDevsR);
    } else if (sumR > sumL && visionEstL.isPresent()) {
      consumer.accept(visionEstL.get().estimatedPose.toPose2d(), visionEstL.get().timestampSeconds, curStdDevsL);
    }
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator,
      Matrix<N3, N1> curStdDevs) {
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
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());

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
    return curStdDevsL;
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
