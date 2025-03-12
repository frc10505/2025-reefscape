/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;


  

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.team10505.robot.Constants.VisionConstants.*;

import java.util.Optional;

import frc.team10505.robot.subsystems.DrivetrainSubsystem;

public class Vision {
/*Cameras */
public final PhotonCamera reefCam = new PhotonCamera("reefCam");
public final PhotonCamera backCam = new PhotonCamera("backCam");

/*field layout */
private final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

/*pose estimators */
private final PhotonPoseEstimator reefCamEstimator = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, kRobotToReefCamTransform);
private final PhotonPoseEstimator backCamEstimator = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, kRobotToBackCamTransform);

/*Simulation */
public final VisionSystemSim visionSim = new VisionSystemSim("Vision Sim");
private final SimCameraProperties cameraProperties = new SimCameraProperties();
private PhotonCameraSim reefCamSim = new PhotonCameraSim(reefCam, cameraProperties);
private PhotonCameraSim backCamSim = new PhotonCameraSim(backCam, cameraProperties);


public Vision() {
    reefCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
   
    //sim stuff
    visionSim.addAprilTags(kFieldLayout);
    visionSim.addCamera(reefCamSim, kRobotToReefCamTransform);
    visionSim.addCamera(backCamSim, kRobotToBackCamTransform);

    cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(35.0);
    cameraProperties.setLatencyStdDevMs(5);
    cameraProperties.setCalibration(kWidthOfCamera, kHeightOfCamera, kCameraFOV);            
}



/*Calculations for pose estimations */
public double lastReefCamEstimateTimestamp = 0.0;

public Optional<EstimatedRobotPose> getReefCamEstimatedPose(){
        Optional<EstimatedRobotPose> reefCamRobotPose = Optional.empty();

        for (PhotonPipelineResult change : reefCam.getAllUnreadResults() ){

            reefCamRobotPose = reefCamEstimator.update(change);
        }
        
        lastReefCamEstimateTimestamp = reefCam.getLatestResult().getTimestampSeconds();

        return reefCamRobotPose;
    }

public double lastBackCamEstimateTimestamp = 0.0;

public Optional<EstimatedRobotPose> getBackCamEstimatedPose(){
        Optional<EstimatedRobotPose> backCamRobotPose = Optional.empty();

        for (PhotonPipelineResult change : backCam.getAllUnreadResults() ){

            backCamRobotPose = backCamEstimator.update(change);
        }
        
        lastBackCamEstimateTimestamp = backCam.getLatestResult().getTimestampSeconds();

        return backCamRobotPose;
    }







public void reset() {
    visionSim.clearAprilTags();
    visionSim.addAprilTags(kFieldLayout);
}

}