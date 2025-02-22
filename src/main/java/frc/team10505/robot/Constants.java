/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
    public final class OperatorInterfaceConstants {
      public final static int kXboxControllerPort = 0;
      public final static int kControlPanelPort = 1;
    }

    public final class VisionConstants {
      public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

      public static final int kWidthOfCamera = 4656;
      public static final int kHeightOfCamera = 3496;
      public static final Rotation2d kCameraFOV = Rotation2d.fromDegrees(90.0);

      public static final String kMod0CameraName = "mod0Camera";
      public static final String kMod1CameraName = "mod1Camera";
      public static final String kMod2CameraName = "mod2Camera";
      public static final String kMod3CameraName = "mod3Camera";


      //TODO
      //TRANFORM 3D VALUES 
      //ARE MADE UP
      //THEY MUST BE CHANGED eventually
      public static final Transform3d kRobotToMod0CameraTransform = new Transform3d(
        new Translation3d(-1.0, 1.0, 0.0),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(135.0))
      );
      public static final Transform3d kRobotToMod1CameraTransform = new Transform3d(
        new Translation3d(1.0, 1.0, 0.0),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(45.0))
      );
      public static final Transform3d kRobotToMod2CameraTransform = new Transform3d(
        new Translation3d(1.0, -1.0, 0.0),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(315.0))
      );
      public static final Transform3d kRobotToMod3CameraTransform = new Transform3d(
        new Translation3d(-1.0, -1.0, 0.0),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(225.0))
      );

      public static final Measure<DistanceUnit> kCameraHeight = Inches.of(4);
      public static final Measure<AngleUnit> kCameraPitch = Degrees.of(45); //23.5

      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
    
    public final class ElevatorConstants {
            // PID Constants
            public final static double KP = 1.0;
            public final static double KI = 0.0;
            public final static double KD = 0.0;

            //Motor IDs
            public final static int kElevatorMotorId = 10;
            public final static int kElevatorFollowerMotorId = 11;
      
            //  ffe Constants
            public final static double KS = 5.0;
            public final static double KG = 2.0;
            public final static double KV = 0.9;
            public final static double KA = 0.1;
      
        // simulation constants
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
      public final static int kIntakeMotorCurrentLimit = 15;

      //Intake speed
      public final static double intakeSpeed = 0.2;   //TODO tune with algae

      public final static double pivotEncoderOffset = 0;
      public final static double pivotEncoderScale = 360;

      // PID Constants
      public final static double KP = 1.0;
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
      public final static double simPivotKG = 1.315; //1.4> -> 1.3<
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
      public final static double kIntakeSpeed = 0.4;
      public final static double kOutakeSpeed = 0.4;
      public final static double kTroughSpeed = 0.2;
    }

  
}