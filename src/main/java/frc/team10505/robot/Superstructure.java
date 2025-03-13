/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
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
    private Vision vision;

    public Superstructure(CoralSubsystem coralSubsys, AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsys, DrivetrainSubsystem drivetrainSubsys, Vision vision){
        this.coralSubsystem = coralSubsys;
        this.algaeSubsystem = algaeSubsys;
        this.elevatorSubsystem = elevatorSubsys;
        this.drivetrainSubsystem = drivetrainSubsys;
        this.vision = vision;
    }  


    public Command intakeCoral(){
        return Commands.sequence(
            coralSubsystem.intake().until(()-> (coralSubsystem.outSensor() && coralSubsystem.inSensor())),
            coralSubsystem.slow().until(()-> (coralSubsystem.outSensor() && !coralSubsystem.inSensor()))
        );
    } 
    public Command outputCoral() {
        return Commands.sequence(
            coralSubsystem.output().until(()-> (!coralSubsystem.outSensor())),
            elevatorSubsystem.setHeight(0.0)//setHeightRun( 0.0).until(() -> (elevatorSubsystem.isNearGoal()))
            );            
        
    }
    public Command outputCoralTrough() {
        return Commands.sequence(
            coralSubsystem.trough().until(()-> (!coralSubsystem.outSensor())),
            coralSubsystem.stop()
        );
    }
    public Command outputTopCoral() {
        return Commands.sequence(
            coralSubsystem.outputTop().until(()-> (!coralSubsystem.outSensor())),
            elevatorSubsystem.setHeight(52.0),
            Commands.waitUntil(()-> (elevatorSubsystem.isNearGoal())),
            elevatorSubsystem.setHeight(0.0)
        );
    }
    public Command grabAlgae() {
        return Commands.sequence(
            algaeSubsystem.intakeReverse(),
            algaeSubsystem.coastPivot()  
        );
    }
    public Command holdAlgae(){
        return Commands.sequence(
            algaeSubsystem.intakeStop(),
            algaeSubsystem.setAngle(-13)
        );
    }


    public Command alignToReef(){
      //  if(vision.reefCam.getLatestResult().hasTargets()){
            return drivetrainSubsystem.alignWithReef();//.until(() -> (drivetrainSubsystem.isNearTarget() | (!vision.reefCam.getLatestResult().hasTargets())));
       // } else{
      //      return Commands.print("No target!!");
      //  }

    }

    //COMMANDS FOR AUTONS
    public Command autoIntakeCoral(){
        return coralSubsystem.autoIntake().until(()-> (coralSubsystem.outSensor() && coralSubsystem.inSensor()));
    } 
    public Command autoOutputCoral() {
        return coralSubsystem.output().until(()-> (!coralSubsystem.outSensor()));
       
    }
    public Command autoOutputCoralTrough() {
        return coralSubsystem.trough().until(()-> (!coralSubsystem.outSensor()));
    }
    public Command autoOutputTopCoral() {
        return coralSubsystem.outputTop().until(()-> (!coralSubsystem.outSensor()));
    }
    public Command autoGrabAlgae() {
        return Commands.sequence(
            algaeSubsystem.intakeReverse(),
            algaeSubsystem.coastPivot()  
        );
    }
    public Command autoHoldAlgae(){
        return algaeSubsystem.intakeStop();
    }

    public Command autoScoreCoralL4(){
        return Commands.sequence(
          elevatorSubsystem.setHeight(48.5),
          Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
          coralSubsystem.output().until(()-> (!coralSubsystem.outSensor()))
        );
    }

    public Command autoScoreCoralL3(){
        return Commands.sequence(
          elevatorSubsystem.setHeight(24.0),
          Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
          Commands.waitSeconds(0.5),
          coralSubsystem.output().until(()-> (!coralSubsystem.outSensor()))
        );
        
    }   public Command autoScoreCoralL2(){
        return Commands.sequence(
          elevatorSubsystem.setHeight(9.0),
          Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
          Commands.waitSeconds(0.5),
          coralSubsystem.output().until(()-> (!coralSubsystem.outSensor()))
        );
    }  
     public Command autoScoreCoralL1(){
        return Commands.sequence(
          elevatorSubsystem.setHeight(0.0),
            Commands.waitUntil(() -> elevatorSubsystem.isNearGoal()),
          coralSubsystem.output().until(()-> (!coralSubsystem.outSensor()))
        );
    }

    public Command autoL4Bump(){
        return Commands.sequence(
            elevatorSubsystem.setHeight(52.0),
            Commands.waitUntil(() -> elevatorSubsystem.isNearGoal())
        );
    }

    public Command autoElevDown(){
        return elevatorSubsystem.setHeight(0.0);
    }
}
