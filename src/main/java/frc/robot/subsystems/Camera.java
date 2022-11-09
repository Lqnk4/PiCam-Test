// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


public class Camera extends SubsystemBase {

  public PhotonCamera camera;

  /** Creates a new Camera. */
  public Camera(PhotonCamera camera) {
    this.camera = camera;
  }

  @Override
  public void periodic() {

    var result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();

    // Get the pipeline latency.
    double latencySeconds = result.getLatencyMillis() / 1000.0;


    // Cool stuff
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();

    //Translation to target
    Transform3d pose = target.getBestCameraToTarget();

    //Target corners from camera feed
    List<TargetCorner> corners = target.getCorners();

    SmartDashboard.putBoolean("Has Targets", hasTargets);
    SmartDashboard.putNumber("Latency (s)", latencySeconds);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Area", area);
    SmartDashboard.putNumber("Skew", skew);
  }
}
