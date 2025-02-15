/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Constants.AlgaeConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem extends SubsystemBase {

    //motor controllers

    private final SparkMax intakeMotor = new SparkMax(AlgaeConstants.kAlgaeIntakeMotorID,MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(AlgaeConstants.kAlgaePivotMotorId, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    //Encoder
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder() ;

    //Controller
    private final PIDController pivotController = new PIDController(AlgaeConstants.KP, AlgaeConstants.KI, AlgaeConstants.KD);

    private double pivotSetpoint = -90;

    //Get encoder
    public double getPivotAngle(){
        return pivotEncoder.getPosition();
    }

    //Calc PID
    public double PIDEffort(){
        return pivotController.calculate(getPivotAngle(),pivotSetpoint);
    }

    public AlgaeSubsystem() {
        configAlgaeSubsys();
    }

    public Command setAngle(double angle){
        return runOnce(() ->{
        pivotSetpoint = angle;
        });
    } 
    public Command holdAngle(){
        return run(() ->{
            pivotMotor.setVoltage(PIDEffort());
        });
    }
    public Command intakeForward(){
        return runOnce(() ->{
            intakeMotor.set(AlgaeConstants.intakeSpeed);
        });
    }
    public Command intakeReverse(){
        return runOnce(() ->{
            intakeMotor.set(-AlgaeConstants.intakeSpeed);
        });
    }
    public Command intakeStop(){
        return runOnce(() ->{
            intakeMotor.set(0);
        });
    }
    


    private void configAlgaeSubsys(){
        //Pivot motor config
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(AlgaeConstants.kPivotMotorCurrentLimit,AlgaeConstants.kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(AlgaeConstants.pivotEncoderScale);                        //Angle encoder scale
        pivotMotorConfig.absoluteEncoder.zeroOffset(AlgaeConstants.pivotEncoderOffset);                                     //Angle encoder offset
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(AlgaeConstants.kIntakeMotorCurrentLimit,AlgaeConstants.kIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

}