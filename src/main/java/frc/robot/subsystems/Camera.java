// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  private static Camera instance = null;

  private PhotonCamera camera;
  private PhotonTrackedTarget lastTarget;
  private double lastTimestamp;
  private boolean hasTarget;

  public static Camera getInstance() {
    if (instance == null)
      instance = new Camera();
    return instance;
    // returns something I think
  }

  /** Creates a new Camera. */
  private Camera() {
    camera = new PhotonCamera("photonvision");
    lastTarget = null;
    lastTimestamp = -1;
    hasTarget = false;
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      hasTarget = true;
      PhotonTrackedTarget target = result.getBestTarget();
      double timestamp = result.getTimestampSeconds();

      // Get horizontal and vertical angles (radians)
      double yaw = Math.toRadians(target.getYaw());
      double pitch = Math.toRadians(target.getPitch());

      // Example: assume object distance from camera is estimated
      double distanceMeters = estimateDistance(pitch);

      // Convert polar coordinates to Cartesian (x forward, y sideways)
      double x = distanceMeters * Math.cos(yaw);
      double y = distanceMeters * Math.sin(yaw);

      if (lastTarget != null && lastTimestamp > 0) {
        double dt = timestamp - lastTimestamp;

        // Previous position
        double lastYaw = Math.toRadians(lastTarget.getYaw());
        double lastPitch = Math.toRadians(lastTarget.getPitch());
        double lastDistance = estimateDistance(lastPitch);

        double lastX = lastDistance * Math.cos(lastYaw);
        double lastY = lastDistance * Math.sin(lastYaw);

        // Velocity components (m/s)
        double vx = (x - lastX) / dt;
        double vy = (y - lastY) / dt;

        double speed = Math.sqrt(vx * vx + vy * vy);

        System.out.printf("Velocity: vx=%.2f m/s, vy=%.2f m/s, speed=%.2f m/s%n", vx, vy, speed);
      }

      lastTarget = target;
      lastTimestamp = timestamp;
    } else {
      hasTarget = false;
    }
  }

  /**
   * Returns the velocity (vx, vy, speed) of the closest target in meters per
   * second.
   * 
   * @param camera        PhotonCamera instance
   * @param lastTarget    previously tracked target (can be null initially)
   * @param lastTimestamp timestamp of the previous frame (seconds)
   * @return double[] {vx, vy, speed} or null if no target available
   */
  public double[] getClosestTargetVelocity() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Pick the closest target (largest area is a common proxy)
      PhotonTrackedTarget target = result.getTargets()
          .stream()
          .max((a, b) -> Double.compare(a.getArea(), b.getArea()))
          .orElse(null);

      if (target == null)
        return null;

      double timestamp = result.getTimestampSeconds();

      // Convert yaw/pitch to radians
      double yaw = Math.toRadians(target.getYaw());
      double pitch = Math.toRadians(target.getPitch());

      // Estimate distance from pitch angle
      double distanceMeters = estimateDistance(pitch);

      // Cartesian coordinates relative to camera
      double x = distanceMeters * Math.cos(yaw);
      double y = distanceMeters * Math.sin(yaw);

      if (lastTarget != null && lastTimestamp > 0) {
        double dt = timestamp - lastTimestamp;
        if (dt <= 0)
          return null;

        double lastYaw = Math.toRadians(lastTarget.getYaw());
        double lastPitch = Math.toRadians(lastTarget.getPitch());
        double lastDistance = estimateDistance(lastPitch);

        double lastX = lastDistance * Math.cos(lastYaw);
        double lastY = lastDistance * Math.sin(lastYaw);

        // Velocity components
        double vx = (x - lastX) / dt;
        double vy = (y - lastY) / dt;
        double speed = Math.sqrt(vx * vx + vy * vy);

        return new double[] { vx, vy, speed };
      }
    }
    return null;
  }

  // Simple distance estimation from pitch angle (replace with your calibration)
  public double distToClosestTarget() {
    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
    if (target == null) 
      return -1;
    // TODO: fix this, ioevno if this works
    return target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
  }

  public double[] relativeAngleToTarget(PhotonTrackedTarget target) {
    double[] ret = {target.getYaw(), target.getPitch()};
    return ret;
  }

  public double[] relativeAngleToTarget() {
    if (!hasTarget) {
      return new double[0];
    }

    return relativeAngleToTarget(lastTarget);
  }
  private double estimateDistance(double lastPitch) {
    // TODO: implement dis
    return 0;
  }
}
