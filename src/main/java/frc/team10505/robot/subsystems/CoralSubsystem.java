package frc.team10505.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Robot;
import frc.team10505.robot.Constants.CoralConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

public class CoralSubsystem extends SubsystemBase {

    //Motor controllers
    private final SparkMax intakeLeft = new SparkMax(CoralConstants.kLeftMotorId, MotorType.kBrushless);
    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private final SparkMax intakeRight = new SparkMax(CoralConstants.kRightMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    //Laser sensors
    private final LaserCan inLaser = new LaserCan(60);
    private final LaserCan outLaser = new LaserCan(61);

    public CoralSubsystem(){
        configCoralSubsys();
    }
    public boolean inSensor(){
        LaserCan.Measurement inMeas =inLaser.getMeasurement();
        return (inMeas.distance_mm < 30.0 && inMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }
    public boolean outSensor(){
        LaserCan.Measurement outMeas =outLaser.getMeasurement();
        return (outMeas.distance_mm < 30.0 && outMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }
    public Robot() {
        intakeRightConfig.inverted(true);
    }
    public Command intake(){
        return run(() -> {
            intakeLeft.set(CoralConstants.kIntakeSpeed);
            intakeRight.set(CoralConstants.kIntakeSpeed);
        });
    }public Command output(){
        return run(() -> {
            intakeLeft.set(CoralConstants.kOutakeSpeed);
            intakeRight.set(CoralConstants.kOutakeSpeed);
        });
    }
     public Command trough(){
        return run(() -> {
            intakeLeft.set(CoralConstants.kTroughSpeed);
            intakeRight.set(CoralConstants.kTroughSpeed*0.9);
        });
    }
     public Command stop(){
        return run(() -> {
            intakeLeft.set(0);
            intakeRight.set(0);
         });
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("inSensor", inSensor());
        SmartDashboard.putBoolean("outSensor", outSensor());
    }

    private void configCoralSubsys(){
        //Left intake config
        intakeLeftConfig.idleMode(IdleMode.kBrake);
        intakeLeftConfig.smartCurrentLimit(CoralConstants.kLeftMotorCurrentLimit,CoralConstants.kLeftMotorCurrentLimit);
        intakeLeft.configure(intakeLeftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        //Right intake config
        intakeRightConfig.idleMode(IdleMode.kBrake);
        intakeRightConfig.smartCurrentLimit(CoralConstants.kRightMotorCurrentLimit, CoralConstants.kRightMotorCurrentLimit);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


}
