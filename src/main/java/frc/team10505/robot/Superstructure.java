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

    public Superstructure(CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsys, ElevatorSubsystem elevatorSubsys, DrivetrainSubsystem drivetrainSubsys){
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
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
            coralSubsystem.stop()
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
            coralSubsystem.stop()
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
            algaeSubsystem.brakePivot()
        );
    }
}
