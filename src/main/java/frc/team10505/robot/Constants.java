/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
  }

  public final class VisionConstants {
    public static final int kWidthOfCamera = 4656;
    public static final int kHeightOfCamera = 3496;
    public static final Rotation2d kCameraFOV = Rotation2d.fromDegrees(90.0);

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // TODO
    // TRANFORM 3D VALUES- make accurate
    public static final Transform3d kRobotToBackCamTransform = new Transform3d(
        new Translation3d(-1.0, 1.0, 0.0),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));
    public static final Transform3d kRobotToReefCamTransform = new Transform3d(
        new Translation3d(-0.254, -0.254, 0.178),
        new Rotation3d(0.0, Units.degreesToRadians(25.0), Units.degreesToRadians(15.0)));//3rd value 45
   

    //positions of things on the field
      public static final Pose2d tag17or8L = new Pose2d(3.026,3.723,Rotation2d.fromDegrees(60));
      public static final Pose2d tag22or9L = new Pose2d(4.973,2.861,Rotation2d.fromDegrees(120));
      public static final Pose2d tag21or10L = new Pose2d(5.74,3.862,Rotation2d.fromDegrees(180));
      public static final Pose2d tag20or11L = new Pose2d(5.257,5.026,Rotation2d.fromDegrees(-120));
      public static final Pose2d tag19or6L = new Pose2d(4.008,5.19,Rotation2d.fromDegrees(-60));
      public static final Pose2d tag18or7L = new Pose2d(3.24,4.19,Rotation2d.fromDegrees(0));
      public static final Pose2d tag17or8R = new Pose2d(4.008,2.861,Rotation2d.fromDegrees(60));
      public static final Pose2d tag22or9R = new Pose2d(5.257,3.026,Rotation2d.fromDegrees(120));
      public static final Pose2d tag21or10R = new Pose2d(5.74,4.19,Rotation2d.fromDegrees(180));
      public static final Pose2d tag20or11R = new Pose2d(4.973,5.19,Rotation2d.fromDegrees(-120));
      public static final Pose2d tag19or6R = new Pose2d(3.723,5.026,Rotation2d.fromDegrees(-60));
      public static final Pose2d tag18or7R = new Pose2d(3.24,3.862,Rotation2d.fromDegrees(0));
  }

  public final class DrivetrainConstants {
    // Alignmnet Constants
    public final static double kStrafeP = 0.01;
    public final static double kStrafeI = 0.00;
    public final static double kStrafeD = 0.005;

    public final static double kTurnP = 0.01;
    public final static double kTurnI = 0.0;
    public final static double kTurnD = 0.005;

    public final static double kDistanceP = 0.01;
    public final static double kDistanceI = 0.0;
    public final static double kDistanceD = 0.005;
    public final static double kTurnSetpoint = 180.0;

    //positions of things on the field
    public static final Pose2d tag17or8L = new Pose2d(3.026,3.723,Rotation2d.fromDegrees(60));
    public static final Pose2d tag22or9L = new Pose2d(4.973,2.861,Rotation2d.fromDegrees(120));
    public static final Pose2d tag21or10L = new Pose2d(5.74,3.862,Rotation2d.fromDegrees(180));
    public static final Pose2d tag20or11L = new Pose2d(5.257,5.026,Rotation2d.fromDegrees(-120));
    public static final Pose2d tag19or6L = new Pose2d(4.008,5.19,Rotation2d.fromDegrees(-60));
    public static final Pose2d tag18or7L = new Pose2d(3.24,4.19,Rotation2d.fromDegrees(0));
    public static final Pose2d tag17or8R = new Pose2d(4.008,2.861,Rotation2d.fromDegrees(60));
    public static final Pose2d tag22or9R = new Pose2d(5.257,3.026,Rotation2d.fromDegrees(120));
    public static final Pose2d tag21or10R = new Pose2d(5.74,4.19,Rotation2d.fromDegrees(180));
    public static final Pose2d tag20or11R = new Pose2d(4.973,5.19,Rotation2d.fromDegrees(-120));
    public static final Pose2d tag19or6R = new Pose2d(3.723,5.026,Rotation2d.fromDegrees(-60));
    public static final Pose2d tag18or7R = new Pose2d(3.24,3.862,Rotation2d.fromDegrees(0));


    // Left side
    public final static double kLeftDistanceSetpoint = 2.0;
    public final static double kLeftYawSetpoint = 2.0;

    // right side
    public final static double kRightDistanceSetpoint = 2.0;
    public final static double kRightYawSetpoint = 2.0;

    public final static double rightDriveLaserDistance = 400.0;
    public final static double leftDriveLaserDistance = 300.0;

  }

  public final class ElevatorConstants {
    // PID Constants
    public final static double KP = 0.32;
    public final static double KI = 0.0;
    public final static double KD = 0.0;

    // Motor IDs
    public final static int kElevatorMotorId = 10;
    public final static int kElevatorMotorCurrentLimit = 40;
    public final static int kElevatorFollowerMotorId = 11;

    // ffe Constants
    public final static double KS = 0.0;
    public final static double KG = 0.4;
    public final static double KV = 19.38;
    public final static double KA = 0.01;

    // simulation constant
    public final static double kMaxHeightMeters = 1.5;
    // sim elevator PID constants
    public final static double simKP = 2;
    public final static double simKI = 0;
    public final static double simKD = 0.01;

    // sim elevator ffe constants
    public final static double simKS = 4.0;
    public final static double simKG = 0;
    public final static double simKV = 0.4;
    public final static double simKA = 0.1;
  }

  public final class AlgaeConstants {
    public final static int kAlgaePivotMotorId = 8;
    public final static int kPivotMotorCurrentLimit = 15;
    public final static int kAlgaeIntakeMotorID = 7;
    public final static int kIntakeMotorCurrentLimit = 25;

    // Intake speed
    public final static double intakeSpeed = 0.5;

    public final static double pivotEncoderOffset = 0;
    public final static double pivotEncoderScale = 360;

    // PID Constants
    public final static double KP = 0.1;
    public final static double KI = 0.0;
    public final static double KD = 0.0;

    // Simulation Constants
    public final static int gearing = 36;
    public final static double lengthMeters = 0.5;
    public final static double massKg = 3.0;

    // sim pivot PID constants
    public final static double simPivotKP = 0.3; // $$0.4
    public final static double simPivotKI = 0;
    public final static double simPivotKD = 0.001;

    // sim pivot ffe constants
    public final static double simPivotKS = 4.0;
    public final static double simPivotKG = 1.315; // 1.4> -> 1.3<
    public final static double simPivotKV = 0.4;
    public final static double simPivotKA = 0.1;
  }

  public final class CoralConstants {
    public final static int kLeftMotorId = 2;
    public final static int kLeftMotorCurrentLimit = 15;
    public final static int kRightMotorID = 3;
    public final static int kRightMotorCurrentLimit = 15;
    public final static int kIntakeInId = 60;
    public final static int kIntakeOutId = 61;
    public final static double kIntakeSpeed = 0.30;
    public final static double kOutakeSpeed = 0.25;
    public final static double kOutakeTopSpeed = 0.05;
    public final static double kTroughSpeed = 0.30;
    public final static double kTroughRightMotorPercentage = 0.9;
  }

}