/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    // public final TalonFX elevatorMotor = new TalonFX(kElevatorMotorId);
    public final TalonFX elevatorMotor = new TalonFX(kElevatorMotorId, "kingKan");

    // public final TalonFX elevatorFollowerMotor = new
    // TalonFX(kElevatorFollowerMotorId);
    public final TalonFX elevatorFollowerMotor = new TalonFX(kElevatorFollowerMotorId, "kingKan");

    // Encoders, Real and Simulated
    private double elevatorEncoderValue = 0.0;

    private double totalEffort;
    // Operator interface
    public final SendableChooser<Double> elevaterHeight = new SendableChooser<>();
    private double height = 0.0;

    // Controls, Actual
    private final PIDController elevatorController = new PIDController(KP, KI,
            KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS,
            KG, KV, KA);


    public boolean usePID = true;

    /* Constructor, runs everything inside during initialization */
    public ElevatorSubsystem() {
        elevatorMotor.setPosition(0.0);

        var motorConfig = new MotorOutputConfigs();

        // set current limits
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.getConfigurator().apply(motorConfig);
        elevatorMotor.getConfigurator().apply(limitConfigs);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
        elevatorFollowerMotor.getConfigurator().apply(limitConfigs);
        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    }

    /* commands to referense */
    // changes our setpoint, which changes our pid calcuations therefore effort to
    // the motor, which happens periodically
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public Command setMotor(double voltage){
        return runEnd(() -> {
            usePID = false;
            elevatorMotor.setVoltage(voltage);
        },
        () -> {
            elevatorMotor.setVoltage(0);
            usePID = true;
        });
    }

    // ONLY to use for testing motor direction
    // public Command testElevator(double voltage){
    // return runEnd(() -> {
    // elevatorMotor.setVoltage(voltage);
    // }, () -> {
    // elevatorMotor.setVoltage(0.0);
    // });
    // }

    /* Calculations */
    public double getElevatorEncoder() {
        return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
    }

    public boolean isNearGoal() {
        return MathUtil.isNear(height, getElevatorEncoder(), 2);
    }

    public boolean issGigh () {
        return getElevatorEncoder() > 30;
    }

    public boolean isAbove(double heightOfChoice){
        return getElevatorEncoder() > heightOfChoice;
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }




    @Override
    public void periodic() {
        elevatorEncoderValue = getElevatorEncoder();
        totalEffort = getEffort();

       // if(usePID){
        elevatorMotor.setVoltage(totalEffort * -1.0);
      //  }

        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoderValue);
        SmartDashboard.putNumber("Elevator Effort", totalEffort);
        SmartDashboard.putNumber("Elevator Height", height);
        SmartDashboard.putBoolean("issGigh", issGigh());

    }

}