// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  // Cameras
  PhotonCamera cameraL = new PhotonCamera("left camera");
  PhotonCamera cameraR = new PhotonCamera("right camera");
  
  // camera offset

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltAndymark);

  private Supplier<Pose2d> currentPose;

  /** Creates a new Vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
