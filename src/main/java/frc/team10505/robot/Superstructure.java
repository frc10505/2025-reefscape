/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;

public class Superstructure {

    private CoralSubsystem coralSubsystem;
    private AlgaeSubsystem algaeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;

    private SwerveRequest.ApplyRobotSpeeds robotDrive = new SwerveRequest.ApplyRobotSpeeds();

    public Superstructure(CoralSubsystem coralSubsys, AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsys,
            DrivetrainSubsystem drivetrainSubsys) {
        this.coralSubsystem = coralSubsys;
        this.algaeSubsystem = algaeSubsys;
        this.elevatorSubsystem = elevatorSubsys;
        this.drivetrainSubsystem = drivetrainSubsys;
    }



    public Command  intakeCoral() {
        return Commands.sequence(
                coralSubsystem.intake().until(() -> (coralSubsystem.outSensor() && coralSubsystem.inSensor())),
                coralSubsystem.slow().until(() -> (coralSubsystem.outSensor() && !coralSubsystem.inSensor())));
    }

    public Command outputCoral() {
        return Commands.sequence(
                coralSubsystem.output().until(() -> (!coralSubsystem.outSensor())),
                elevatorSubsystem.setHeight(0.0)// setHeightRun( 0.0).until(() -> (elevatorSubsystem.isNearGoal()))
        );

    }

    public Command outputCoralTrough() {
        return Commands.sequence(
                coralSubsystem.trough().until(() -> (!coralSubsystem.outSensor())),
                coralSubsystem.stop());
    }

    public Command outputTopCoral() {
        return Commands.sequence(
                coralSubsystem.outputTop().until(() -> (!coralSubsystem.outSensor())),
                elevatorSubsystem.setHeight(54.5), //was 53.0
                Commands.waitUntil(() -> (elevatorSubsystem.isNearGoal())),
                 elevatorSubsystem.setHeight(0.0));
    }

    public Command grabAlgae() {
        return Commands.sequence(
                algaeSubsystem.intakeReverse(),
                algaeSubsystem.coastPivot());
    }

    public Command holdAlgae() {
        return Commands.sequence(
                algaeSubsystem.intakeStop(),
                algaeSubsystem.setAngle(-13));
    }

    public Command manualL4Bump() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(54.0),// 54.5
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.setStop(), 
                elevatorSubsystem.setHeight(0.0)
                );
    }

    // public Command autoTwist(){
    //     return Commands.run(() -> {
    //     if((!drivetrainSubsystem.seesLeftSensor()) && (drivetrainSubsystem.seesRightSensor())){
    //          drivetrainSubsystem.setRobotSpeeds(0.2, 0.0, 0.3);
    //     } else if ((drivetrainSubsystem.seesLeftSensor()) && (!drivetrainSubsystem.seesRightSensor())){
    //          drivetrainSubsystem.setRobotSpeeds(0.2, 0.0, -0.3);
    //     }else {
    //          drivetrainSubsystem.setRobotSpeeds(0.2, 0, 0);
    //     }
    // });

    public double dist;

    public Command autoTwist(){
       // return Commands.run(() -> {
      var currentTwist = Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60);

        if(Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60) > 30){
             dist = Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60) - 60;
        } else{
            dist = Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60);
        }

          return drivetrainSubsystem.setRobotSpeeds(0.0, 0.0, (dist / 20));//.until(() -> isNearGoalTwist(0, Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60))),
   // });

    //   if (currentTwist > 45){
    //     return Commands.sequence(
    //         drivetrainSubsystem.setRobotSpeeds(0.0, 0.0, -1).until(() -> isNearGoalTwist(60, Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60))),
    //         drivetrainSubsystem.stop());
    // } else if(currentTwist<15){
    //     return Commands.sequence(
    //      drivetrainSubsystem.setRobotSpeeds(0.0, 0.0, 1).until(() -> isNearGoalTwist(0, Math.abs(drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees() % 60))),
    //      drivetrainSubsystem.stop());
    //   } else{
    //     return Commands.print("NOT CLOSE ENOUGH");//drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0, 0.0))).until(() -> drivetrainSubsystem.seesLeftSensor() && drivetrainSubsystem.seesRightSensor()); //0.3
    //   }

    }




    public boolean isNearGoalTwist(double goal, double current){
        return MathUtil.isNear(goal, current, 3);
    }



    // COMMANDS FOR AUTONS
    public Command autoIntakeCoral() {
        return coralSubsystem.autoIntake().until(() -> (coralSubsystem.outSensor() && coralSubsystem.inSensor()));
    }

    public Command autoOutputCoral() {
        return coralSubsystem.output().until(() -> (!coralSubsystem.outSensor()));

    }

     public Command autoOutputCoralTrough() {
        return coralSubsystem.trough().until(() -> (!coralSubsystem.outSensor()));
    }

    public Command autoOutputTopCoral() {
        return coralSubsystem.outputTop().until(() -> (!coralSubsystem.outSensor()));
    }

    public Command autoGrabAlgae() {
        return Commands.sequence(
                algaeSubsystem.intakeReverse(),
                algaeSubsystem.coastPivot());
    }

    public Command autoHoldAlgae() {
        return algaeSubsystem.intakeStop();
    }

    public Command autoScoreCoralL4() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(48.5),//49.5 -> shoots over top//48.5
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.autoSetIntake(),
                Commands.waitUntil(() -> (!coralSubsystem.outSensor())));
    }

    public Command autoScoreCoralL3() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(24.0),
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                Commands.waitSeconds(0.5),
                coralSubsystem.output().until(() -> (!coralSubsystem.outSensor())));

    }

    public Command autoScoreCoralL2() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(8.0),
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                Commands.waitSeconds(0.5),
                coralSubsystem.output().until(() -> (!coralSubsystem.outSensor()))//,
                // coralSubsystem.setOutput(),
                // Commands.race(
                //     Commands.waitUntil(()->(!coralSubsystem.outSensor())),
                //     Commands.waitSeconds(4)
                // ),
                // coralSubsystem.setStop()
                );
    }

    public Command dropCoral(){
        return coralSubsystem.output().until(() -> (!coralSubsystem.outSensor()));

    }

    public Command autoScoreCoralL1() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(0.0),
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.output().until(() -> (!coralSubsystem.outSensor())));
    }

    public Command autoL4Bump() {
        return Commands.sequence(
                elevatorSubsystem.setHeight(54.0),// 54.5
                Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
                coralSubsystem.setStop());
    }

    public Command autoElevDown() {
        return elevatorSubsystem.setHeight(0.0);
    }

    public Command autoAlignLeft(){
        return Commands.sequence(
             drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.3, 0.0))).until(() -> !drivetrainSubsystem.seesLeftSensor()),
             drivetrainSubsystem.stop()
             // Commands.none()
        );
    }



    // public Command autoAlignRight(){
    //     return Commands.sequence(
    //         drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, -0.35, 0.0))).until(() -> !drivetrainSubsystem.seesRightSensor()), //0.3
    //         drivetrainSubsystem.stop()
    //         //drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))).until(() -> !drivetrainSubsystem.seesRightSensor()),
    //         //Commands.none()
    //         );
    //     }

        // public Command autoDriveForwardBothSensors(){
        //     return Commands.sequence(
        //          drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0.0, 0.0))).until(() ->( drivetrainSubsystem.seesRightSensor() && drivetrainSubsystem.seesLeftSensor())),
        //          drivetrainSubsystem.stop()
        //          // Commands.none()
        //     );
        // }

        public Command autoDriveForward(){
            return Commands.sequence(
                 drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0.0, 0.0))).until(() ->(drivetrainSubsystem.seesLeftSensor())),
                 drivetrainSubsystem.stop()
                 // Commands.none()
            );
        }

        // public Command autoDriveForwardTillSeesRight(){
        //     return Commands.sequence(
        //          drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0.0, 0.0))).until(() ->( drivetrainSubsystem.seesRightSensor())),
        //          drivetrainSubsystem.stop()
        //          // Commands.none()
        //     );
        // }


        public Command setPose(double x, double y, double rot) {
            return Commands.runOnce(() -> {
                drivetrainSubsystem.resetPose(new Pose2d(x, y, new Rotation2d(rot)));
            });
        }


}

