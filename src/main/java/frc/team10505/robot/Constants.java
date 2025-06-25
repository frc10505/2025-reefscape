/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
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

    // Left side
    public final static double kLeftDistanceSetpoint = 2.0;
    public final static double kLeftYawSetpoint = 2.0;

    // right side
    public final static double kRightDistanceSetpoint = 2.0;
    public final static double kRightYawSetpoint = 2.0;

    public final static double rightDriveLaserDistance = 290.0;//210//$190//235 BAD  //$260
    public final static double leftDriveLaserDistance = 290;//180    //$260

    public final static double autonRightDriveLaserDistance = 325.0;//320
    public final static double autonLeftDriveLaserDistance = 290;//$280 before laser can fall

    public final static double closeRightDriveLaserDistance = 235.0; //180
    public final static double closeLeftDriveLaserDistance = 250.0; //180
 

  }

  public final class ElevatorConstants {
    // PID Constants
    public final static double KP = 0.467;
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
    public final static double intakeSpeed = 0.5;//0.5
    public final static double intakeSlowSpead = 0.3; //0.3
    public final static double sigmaSpead = 0.2;

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
    public final static double kIntakeSpeed = 0.37;//.4  Donny is sigma walter is no buns
    public final static double kLeftL1Speed = 0.5;//.2
    public final static double kRightL1Speed = 0.25;//.4
    public final static double kOutakeSpeed = 0.25;
    public final static double kOutakeTopSpeed = 0.2;//.1
    public final static double kTroughSpeed = 0.30;
    public final static double kTroughRightMotorPercentage = 0.9;
  }

}