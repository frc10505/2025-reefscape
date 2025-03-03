/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // motor controllers

    public final static SparkMax intakeMotor = new SparkMax(AlgaeConstants.kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(AlgaeConstants.kAlgaePivotMotorId, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Encoder
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private double encoderValue;
    private double absoluteOffset = 0.0;

    // Controller
    private final PIDController pivotController = new PIDController(AlgaeConstants.KP, AlgaeConstants.KI,
            AlgaeConstants.KD);

    private double pivotSetpoint = -90;

    // Get encoder
    public double getPivotEncoder() {
        return (pivotEncoder.getPosition() - absoluteOffset) * -1.0;// TODO adjust
    }

    // Calc PID
    public double PIDEffort() {
        return pivotController.calculate(getPivotEncoder(), pivotSetpoint);
    }

    public AlgaeSubsystem() {
        configAlgaeSubsys();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
    }

    public Command setAngle(double angle) {
        return runOnce(() -> {
            pivotSetpoint = angle;
        });
    }

   public boolean coasting = false;

    public Command holdAngle() {
        return run(() -> {
            if (!coasting) {
            pivotMotor.setVoltage(PIDEffort());
            }
        });
    }
    public Command coastPivot(){
        return runOnce(() ->{
            pivotMotorConfig.idleMode(IdleMode.kCoast);
            coasting = true;
        });
    }
    public Command brakePivot(){
        return runOnce(() ->{
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotSetpoint = getPivotEncoder();
            coasting = false;
        });
    }

    public Command intakeForward() {
        return runOnce(() -> {
            intakeMotor.set(AlgaeConstants.intakeSpeed);
        });
    }

    public Command intakeReverse() {
        return runOnce(() -> {
            intakeMotor.set(-AlgaeConstants.intakeSpeed);
        });
    }

    public Command intakeStop() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        encoderValue = getPivotEncoder();
        SmartDashboard.putNumber(" Pivot Encoder", encoderValue);
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
    }

    private void configAlgaeSubsys() {
        // Pivot motor config
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(AlgaeConstants.kPivotMotorCurrentLimit,
                AlgaeConstants.kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(AlgaeConstants.pivotEncoderScale); // Angle encoder
                                                                                                     // scale
        pivotMotorConfig.absoluteEncoder.zeroOffset(AlgaeConstants.pivotEncoderOffset); // Angle encoder offset
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(AlgaeConstants.kIntakeMotorCurrentLimit,
                AlgaeConstants.kIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}