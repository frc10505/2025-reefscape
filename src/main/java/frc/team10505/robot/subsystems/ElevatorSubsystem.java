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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public final TalonFX elevatorMotor = new TalonFX(kElevatorMotorId);
    public final TalonFX elevatorFollowerMotor = new TalonFX(kElevatorFollowerMotorId);

    // Encoders, Real and Simulated
    private double simElevatorEncoder;
    private double elevatorEncoderValue = 0.0;

    private double simTotalEffort = 0.0;
    private double totalEffort;
    // Operator interface
    public final SendableChooser<Double> elevaterHeight = new SendableChooser<>();
    private double height = 0.0;

    // Controls, Actual
    private final PIDController elevatorController = new PIDController(KP, KI,
            KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS,
            KG, KV, KA);

    // Controls, Simulated
    private final PIDController simElevatorController = new PIDController(simKP,
            simKI, simKD);
    private final ElevatorFeedforward simElevatorFeedforward = new ElevatorFeedforward(simKS,
            simKG, simKV, simKA);

    // Simulation
    private final ElevatorSim elevatorSim = new ElevatorSim(simKV, simKA,
            DCMotor.getKrakenX60(2), 0, kMaxHeightMeters, true, 0.1);
    // private final SingleJointedArmSim elevatorSim = new
    // SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 6, 0.5, 1.57,
    // 1.58, false, Math.PI / 180);
    public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    public final MechanismLigament2d elevatorViz = elevatorRoot
            .append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));


    /*Constructor, runs everything inside during initialization */
    public ElevatorSubsystem() {
        //for simulation
        SmartDashboard.putData("Elevator Viz", elevatorSimMech);

        //
        elevatorMotor.setPosition(0.0);

        var motorConfig = new MotorOutputConfigs();

        //set current limits
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable=true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.getConfigurator().apply(motorConfig);
        elevatorMotor.getConfigurator().apply(limitConfigs);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
        elevatorFollowerMotor.getConfigurator().apply(limitConfigs);
        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    }


    /*commands to referense */
    //the only command we need to referense
    //changes our setpoint, which changes our pid calcuations therefore effort to the motor, which happens periodically
    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

     //ONLY to use for testing motor direction
    // public Command testElevator(double voltage){
    // return runEnd(() -> {
    // elevatorMotor.setVoltage(voltage);
    // }, () -> {
    // elevatorMotor.setVoltage(0.0);
    // });
    // }

    /*Calculations */
    public double getElevatorEncoder() {
        return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751*2) / 12.0 ) * -1.0;
    }

       public boolean isNearGoal(){
        return MathUtil.isNear(height, getElevatorEncoder(), 1);
    }

    public double simGetEffort() {
        return simTotalEffort = ((simElevatorFeedforward.calculate(0, 0))
                + (simElevatorController.calculate(simElevatorEncoder, height)));
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }

    @Override
    public void periodic() {
        elevatorEncoderValue = getElevatorEncoder();
        totalEffort = getEffort();
        elevatorMotor.setVoltage(totalEffort * -1.0);

        SmartDashboard.putNumber("ElevatorHeight", elevatorEncoderValue);
        SmartDashboard.putNumber("Elevator Effort", totalEffort);
        SmartDashboard.putNumber("Elevator Height", height);

       
        
        //Stuff for simulation
       // might not be functional still, probably not necessary either
        SmartDashboard.putNumber("Sim Elevator Encoder", simElevatorEncoder);
        SmartDashboard.putNumber("Sim Elevator total Effort", simTotalEffort);

        simElevatorEncoder = elevatorViz.getLength();
        simTotalEffort = simGetEffort();
    }

}