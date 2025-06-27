/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.AlgaeConstants.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem extends SubsystemBase {

    // motor controllers
    public final SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    // Encoder
    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private double encoderValue;
    private double absoluteOffset = 180.0;

    // Controller
    private final PIDController pivotController = new PIDController(KP, KI,
            KD);

    private double pivotSetpoint = -90; // 90

    public boolean coasting = false;

    /*Our constructor */
    public AlgaeSubsystem() {
        configAlgaeSubsys();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
    }

    /*Pivot calculation methods */
    // Get encoder
    public double getPivotEncoder() {
        return (-pivotEncoder.getPosition() + absoluteOffset);
    }

    // Calc PID
    public double PIDEffort() {
        return pivotController.calculate(getPivotEncoder(), pivotSetpoint);
    }

    /*pivot commands to reference */
    public Command setAngle(double angle) {
        return runOnce(() -> {
            pivotSetpoint = angle;
        });
    }

    public Command setVoltage(double voltage) {
        return run(() ->{
            pivotMotor.setVoltage(voltage);
        });
    }

    //command we set to as the default command
        /*NOTE that defaul command has been problematic in past years -
            if it becomes an issue, set the pivot motor to pid effort periodically*/
    public Command holdAngle() {
        return run(() -> {
            if (!coasting) {
                pivotMotor.setVoltage(PIDEffort());
            }
        });
    }

    //Only use when testing motor direction, not useful with pid(the motor will only stop for 0.02 s)
    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kCoast);//stops pivot motor from being set to calculated pid effort
            coasting = true;
        });
    }

    public Command brakePivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotSetpoint = getPivotEncoder();
            coasting = false;
        });
    }

    /*intake commands to reference */
    public Command intakeForward() {
        return runOnce(() -> {
            intakeMotor.set(intakeSpeed);
        });
    } 

    public Command runIntakeForward() {
        return runEnd(() -> {
            intakeMotor.set(intakeSpeed);
        }, () -> {
            intakeMotor.set(0);
        });
    }

    public Command intakeForwardSlower() {
        return runOnce(() -> {
            intakeMotor.set(intakeSlowSpead);
        });
    }

    public Command intakeForwardSlowest() {
        return runOnce(() -> {
            intakeMotor.set(sigmaSpead);
        });
    }

    public Command intakeReverse() {
        return runOnce(() -> {
            intakeMotor.set(-intakeSpeed);
        });
    }

    public Command runIntakeReverse() {
        return runEnd(() -> {
            intakeMotor.set(-intakeSpeed);
        }, () -> {
            intakeMotor.set(0);
        });
    }

    public Command intakeStop() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        encoderValue = getPivotEncoder();
        SmartDashboard.putNumber(" Pivot Encoder", encoderValue);
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
    }

    /*configurations to be called in the constructor,
     * runs once during init,
     * here it is used to configure motor settings
     */
    private void configAlgaeSubsys() {
        // Pivot motor config
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit,
                kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale); // Angle encoder scale
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset); // Angle encoder offset
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(kIntakeMotorCurrentLimit,
                kIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}