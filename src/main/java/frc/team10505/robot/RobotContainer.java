/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgeaSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.team10505.robot.Constants.OperatorInterfaceConstants.*;
import static frc.team10505.robot.Constants.VisionConstants.*;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator interfaces */
    private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
    // private final CommandXboxController controlPanel = new CommandXboxController(kControlPanelPort);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
    private final AlgeaSubsystem algeaSubsys = new AlgeaSubsystem();

    /* Camerae */
    private static final Camera mod0Camera = new Camera(kMod0CameraName, kRobotToMod0CameraTransform);
    private static final Camera mod1Camera = new Camera(kMod1CameraName, kRobotToMod1CameraTransform);
    private static final Camera mod2Camera = new Camera(kMod2CameraName, kRobotToMod2CameraTransform);
    private static final Camera mod3Camera = new Camera(kMod3CameraName, kRobotToMod3CameraTransform);

    /* Simulation */
    private final Vision visionSimulation = new Vision(() -> drivetrainSubsys.getState().Pose, drivetrainSubsys);
    //   .addCamera(mod0Camera)
    //   .addCamera(mod1Camera)
    //   .addCamera(mod2Camera)
    //   .addCamera(mod3Camera);

    /* Autonomous */
    private  final SendableChooser<Command> autoChooser;// = new SendableChooser<>();
    
        public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        configDefaultCommands();
        configButtonBindings();
        configAutonomous();
    }

    /**
     * Function that is called in the constructor where we configure default
     * commands for the subsytems.
     */
    private void configDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if (Utils.isSimulation()) {
            drivetrainSubsys.setDefaultCommand(
                drivetrainSubsys.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getRawAxis(1) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            );
        } else {
            drivetrainSubsys.setDefaultCommand(
                drivetrainSubsys.applyRequest(() ->
                    drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
        }
    }

    /**
     * Function that is called in the constructor where we configure operator
     * interface button bindings.
     */
    private void configButtonBindings() {
        if (Utils.isSimulation()) {         
       
            joystick.button(1).whileTrue(elevatorSubsys.setHeight(0.0));
            joystick.button(2).whileTrue(elevatorSubsys.setHeight(1.0));
            joystick.button(3).whileTrue(elevatorSubsys.setHeight(2.5));
            joystick.button(4).whileTrue(elevatorSubsys.setHeight(4.0));
        } else {
            xboxController.a().whileTrue(drivetrainSubsys.applyRequest(() -> brake));
            xboxController.b().whileTrue(drivetrainSubsys.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xboxController.back().and(xboxController.y()).whileTrue(drivetrainSubsys.sysIdDynamic(Direction.kForward));
        xboxController.back().and(xboxController.x()).whileTrue(drivetrainSubsys.sysIdDynamic(Direction.kReverse));
        xboxController.start().and(xboxController.y()).whileTrue(drivetrainSubsys.sysIdQuasistatic(Direction.kForward));
        xboxController.start().and(xboxController.x()).whileTrue(drivetrainSubsys.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xboxController.leftBumper().onTrue(drivetrainSubsys.runOnce(() -> drivetrainSubsys.seedFieldCentric()));

        drivetrainSubsys.registerTelemetry(logger::telemeterize);
        }
    }

    /**
     * Function that returns the currently selected autonomous routine in the
     * SendableChooser.
     * 
     * @return Currently selected autonomous routine.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configAutonomous() {
        SmartDashboard.putData(autoChooser);
    }

    public Pose2d getPose() {
        return drivetrainSubsys.getState().Pose;
    }

    public void resetSimulation() {
        visionSimulation.reset();
    }

    public void updateSimulation() {
      visionSimulation.update(getPose());
    }

}