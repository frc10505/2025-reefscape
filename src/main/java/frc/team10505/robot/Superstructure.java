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

    private SwerveRequest.ApplyRobotSpeeds autoRobotDrive = new SwerveRequest.ApplyRobotSpeeds();

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
      // if (elevatorSubsystem.issGigh()) {
            return Commands.sequence(
                coralSubsystem.outputTop().until(() -> (!coralSubsystem.outSensor())),
                elevatorSubsystem.setHeight(54.5), //was 53.0
                Commands.waitUntil(() -> (elevatorSubsystem.isNearGoal())),
                elevatorSubsystem.setHeight(0.00));
        // } else{
        //     return Commands.print("Cooper struggles with driving due to a lack of focus, poor decision-making, and inability to judge distances. His reaction times are slow, leading to frequent mistakes. He is a very bad BAD BOY. and maybe slow w  \r\n" +
        //                                 "c( . . )o     (\r\n" + //
        //                         "  (   (    -    )   )\r\n" + //
        //                         "  \\  \\_/`-----'   /  \r\n" + //
        //                         "   /  /    |     (   \r\n" + //
        //                         "  (   )  |   )  (  (  \r\n" + //
        //                         "  `-`    `-`    `-` ` ");
        // }
   
    }

    public Command bombsAway(){
        return Commands.sequence(
            //TODO test the set motor cmd, if we have time are we're feeling gutsy
                //NOTE- we'd have to add in the if/else statement in the elev periodic
            //elevatorSubsystem.setMotor(2).until(() -> elevatorSubsystem.getElevatorEncoder() >50),

            elevatorSubsystem.setHeight(55.5),
            Commands.waitUntil(() -> (elevatorSubsystem.getElevatorEncoder() > 42.5)),//42//EDIT VALUE IRL
            //algaeSubsystem.setAngle(-20),
            algaeSubsystem.setVoltage(-1.5).withTimeout(0.05),//-0.5, 0.5, -1.5//-3.5,0.3
           // Commands.waitSeconds(0.3),//.54
           // algaeSubsystem.intakeStop(),
           algaeSubsystem.setVoltage(5.0).until(() -> algaeSubsystem.getPivotEncoder() > 50),
           algaeSubsystem.intakeSkibaglagae(),
           algaeSubsystem.setAngle(90)//sigma


           

            );
    }


    public Command regurgitateAlgae(){
        return Commands.sequence(
        algaeSubsystem.intakeSkibaglagae().withTimeout(0.3),
        algaeSubsystem.intakeStop()
        );
    }

    public Command takeCover(){
        return Commands.sequence(
            elevatorSubsystem.setHeight(0.0),
            algaeSubsystem.setAngle(-90),
            algaeSubsystem.intakeStop()

        );

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
             drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.6, 0.0))).until(() -> !drivetrainSubsystem.seesLeftSensor()),
             drivetrainSubsystem.stop()
             // Commands.none()
        );
    }



    public Command autoAlignRight(){
        return Commands.sequence(
            drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(0.0, -0.75, 0.0))).until(() -> !drivetrainSubsystem.seesRightSensor()), //0.3
            drivetrainSubsystem.stop()
            //drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))).until(() -> !drivetrainSubsystem.seesRightSensor()),
            //Commands.none()
            );
        }

        // public Command autoDriveForwardBothSensors(){
        //     return Commands.sequence(
        //          drivetrainSubsystem.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0.0, 0.0))).until(() ->( drivetrainSubsystem.seesRightSensor() && drivetrainSubsystem.seesLeftSensor())),
        //          drivetrainSubsystem.stop()
        //          // Commands.none()
        //     );
        // }

        public Command autoDriveForward(){
            return Commands.sequence(
                 drivetrainSubsystem.applyRequest(() -> autoRobotDrive.withSpeeds(new ChassisSpeeds(1, 0.0, 0.0))).until(() ->(drivetrainSubsystem.seesLeftSensor() | drivetrainSubsystem.seesRightSensor())),
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


        // public Command setPose(double x, double y, double rot) {
        //     return Commands.runOnce(() -> {
        //         drivetrainSubsystem.resetPose(new Pose2d(x, y, new Rotation2d(rot)));
        //     });
        // }


}

