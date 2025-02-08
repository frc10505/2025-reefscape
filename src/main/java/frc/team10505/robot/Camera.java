/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import static frc.team10505.robot.Constants.VisionConstants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

public class Camera {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private PhotonCameraSim simCamera;

    private double lastEstimateTimestamp = 0.0;

    public Camera(String cameraName, Transform3d robotToCameraTransform) {
        photonCamera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCameraTransform);
        simCamera = new PhotonCameraSim(photonCamera);

        configCamera();
    }

    private PhotonPipelineResult getLatestResult() {
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        return (results.size() == 0) ? new PhotonPipelineResult() : results.get(results.size() - 1);
    }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var visionEstimate = photonPoseEstimator.update(this.getLatestResult());
        double latestTimestamp = getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;

        if (newResult) lastEstimateTimestamp = latestTimestamp;

        return (Optional<EstimatedRobotPose>) visionEstimate;
    }

    public double getTargetHeight() {
        double targetHeight;
        
        var result = getLatestResult();
    
        if (result.hasTargets()) {
          targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
        } else {
          targetHeight = 0.0;
        }
    
        return targetHeight;
    }
    
    public double getTargetDistance() {
        double targetDistance;

        var result = getLatestResult();

        if (result.hasTargets()) {
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
              kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
        } else {
            targetDistance = 0.0;
        }

        return targetDistance;
    }

    public PhotonCameraSim getSimulation(SimCameraProperties cameraProperties) {
        simCamera = new PhotonCameraSim(photonCamera, cameraProperties);
        return simCamera;
    }

    public Transform3d getRobotToCameraTransform() {
        return photonPoseEstimator.getRobotToCameraTransform();
    }

    private void configCamera() {
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
}