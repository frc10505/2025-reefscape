/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Constants;
import frc.team10505.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public final TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorId);
    public final TalonFX elevatorFollowerMotor = new TalonFX(Constants.ElevatorConstants.kElevatorFollowerMotorId);

    // Encoders, Real and Simulated
    private double simElevatorEncoder;
    private double elevatorEncoderValue = 0.0;

    private double simTotalEffort = 0.0;
    private double totalEffort;
    // Operator interface
    public final SendableChooser<Double> elevaterHeight = new SendableChooser<>();
    private double height = 0.0;

    // Controls, Actual
    private final PIDController elevatorController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI,
            ElevatorConstants.KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS,
            ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

    // Controls, Simulated
    private final PIDController simElevatorController = new PIDController(ElevatorConstants.simKP,
            ElevatorConstants.simKI, ElevatorConstants.simKD);
    private final ElevatorFeedforward simElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.simKS,
            ElevatorConstants.simKG, ElevatorConstants.simKV, ElevatorConstants.simKA);

    // Simulation
    private final ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.simKV, ElevatorConstants.simKA,
            DCMotor.getKrakenX60(2), 0, ElevatorConstants.kMaxHeightMeters, true, 0.1);
    // private final SingleJointedArmSim elevatorSim = new
    // SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 6, 0.5, 1.57,
    // 1.58, false, Math.PI / 180);
    public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    public final MechanismLigament2d elevatorViz = elevatorRoot
            .append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));

    public ElevatorSubsystem() {
        SmartDashboard.putData("Elevator Viz", elevatorSimMech);

        elevatorMotor.setPosition(0.0);

        var motorConfig = new MotorOutputConfigs();

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.getConfigurator().apply(motorConfig);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
    }

    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public double getElevatorEncoder() {
        return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 2.15) / 12.0 ) * -1.0;
    }

    public double simGetEffort() {
        return simTotalEffort = ((simElevatorFeedforward.calculate(0, 0))
                + (simElevatorController.calculate(simElevatorEncoder, height)));
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }
    // public Command testElevator(double voltage){
    // return runEnd(() -> {
    // elevatorMotor.setVoltage(voltage);
    // }, () -> {
    // elevatorMotor.setVoltage(0.0);
    // });
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Motor Output",
        // elevatorMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Sim Elevator Motor Output",
        // simElevatorMotor.getMotorVoltage());
        // SmartDashboard.putNumber("Sim Elevator Encoder Inches", simElevatorEncoder);
        // SmartDashboard.putNumber("Elevator Height", height);
        // SmartDashboard.putNumber("ElevatorSim.getPosiitonMeters()",
        // elevatorSim.getPositionMeters());
        // SmartDashboard.putNumber("Sim Elevator total Effort", simTotalEffort);
        // SmartDashboard.putNumber("Sim Elevator PID Effort",
        // simElevatorController.calculate(simElevatorEncoder, height));

        elevatorEncoderValue = getElevatorEncoder();
        SmartDashboard.putNumber("ElevatorHeight", elevatorEncoderValue);
        // simElevatorEncoder = elevatorViz.getLength();
        // elevatorEncoder = elevatorMotor.getPosition().getValueAsDouble(); // TODO <-
        // figure out when using real robot
        // simTotalEffort = simGetEffort();
        totalEffort = getEffort();

        elevatorMotor.setVoltage(totalEffort * -1.0);
        SmartDashboard.putNumber("Control Effort", totalEffort);
    }

}