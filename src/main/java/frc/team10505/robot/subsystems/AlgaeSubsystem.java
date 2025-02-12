/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Constants.AlgaeConstants;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeSubsystem{

    //motor controllers

    public final SparkMax intakeMotor = new SparkMax(AlgaeConstants.kAlgaeIntakeMotorID,MotorType.kBrushless);
    public final SparkMax pivotMotor = new SparkMax(AlgaeConstants.kAlgaePivotMotorId, MotorType.kBrushless);
  //  public final SparkMaxAlternateEncoder pivotEncoder = new SparkMaxAlternateEncoder(SparkMAxpivotMotor)
}