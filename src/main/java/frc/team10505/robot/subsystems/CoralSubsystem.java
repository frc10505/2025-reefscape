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
        return (inMeas.distance_mm < 50.0 && inMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }
    public boolean outSensor(){
        LaserCan.Measurement outMeas =outLaser.getMeasurement();
        return (outMeas.distance_mm < 50.0 && outMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }
        public Command intake(){
            return runEnd(() -> {
                intakeLeft.set(CoralConstants.kIntakeSpeed);
                intakeRight.set(CoralConstants.kIntakeSpeed);
            },
            () -> {
                intakeLeft.set(0);
                intakeRight.set(0);
            });
        }
        public Command output(){
            return runEnd(() -> {
                intakeLeft.set(CoralConstants.kOutakeSpeed);
                intakeRight.set(CoralConstants.kOutakeSpeed);
            },
            () -> {
                intakeLeft.set(0);
                intakeRight.set(0);
            });
        }
        public Command outputTop(){
            return runEnd(() -> {
                intakeLeft.set(CoralConstants.kOutakeTopSpeed);
                intakeRight.set(CoralConstants.kOutakeTopSpeed);
        },
        () -> {
            intakeLeft.set(0);
            intakeRight.set(0);
        });
        }
         public Command trough(){
            return runEnd(() -> {
                intakeLeft.set(CoralConstants.kTroughSpeed);
                intakeRight.set(CoralConstants.kTroughSpeed*CoralConstants.kTroughRightMotorPercentage);
            },
            () -> {
                intakeLeft.set(0);
                intakeRight.set(0);
            });
        }
     public Command stop(){
        return run(() -> {
            intakeLeft.set(0);
            intakeRight.set(0);
         });
    }
    public Command slow(){
        return runEnd(() -> {
            intakeLeft.set(0.05);
            intakeRight.set(0.05);
        },
        () -> {
            intakeLeft.set(0);
            intakeRight.set(0);
        });
    }

    public Command autoIntake(){
        return runEnd(() -> {
            intakeLeft.set(CoralConstants.kIntakeSpeed);
            intakeRight.set(CoralConstants.kIntakeSpeed);
        },
        () -> {
            slow().until(() ->(outSensor() && !inSensor()));
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
      intakeRightConfig.inverted(true);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


}
