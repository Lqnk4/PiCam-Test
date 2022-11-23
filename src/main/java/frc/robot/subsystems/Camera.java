// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.camera.*;


public class Camera extends SubsystemBase {

  public PhotonCamera camera;

  /** Creates a new Camera. */
  public Camera(PhotonCamera camera) {
    this.camera = camera;
  }

  @Override
  public void periodic() {


  }

  public void getTargetData() {
    var result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();

    // Get the pipeline latency.
    double latencySeconds = result.getLatencyMillis() / 1000.0;

    if (hasTargets) {


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

  public List<Double> getTargetDistances() {
    var result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    PhotonTrackedTarget bestTarget = result.getBestTarget();


    // Get the pipeline latency.
    double latencySeconds = result.getLatencyMillis() / 1000.0;

    List<Double> distances = new ArrayList<>();

    if(hasTargets) {
      for(PhotonTrackedTarget target : targets) {



        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(target.getPitch()));

        distances.add(range);

      }
    }

    return distances;

  }





}
