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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.Superstructure;
import frc.team10505.robot.Vision;
import static edu.wpi.first.units.Units.*;
import static frc.team10505.robot.Constants.OperatorInterfaceConstants.*;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator interfaces */
    private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
    private final CommandXboxController xboxController2 = new CommandXboxController(kControlPanelPort);
    // private final CommandXboxController controlPanel = new
    // CommandXboxController(kControlPanelPort);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final CoralSubsystem coralSubsys = new CoralSubsystem();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();

    // private final Vision vision = new Vision(drivetrainSubsys);
    /* Superstructure */
    private final Superstructure superStructure = new Superstructure(coralSubsys, algaeSubsys, elevatorSubsys,
            drivetrainSubsys);

    /* Autonomous */
    private final SendableChooser<Command> autoChooser;// = new SendableChooser<>();

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
                    drivetrainSubsys.applyRequest(() -> drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed) // Drive
                                                                                                                // forward
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // Y
                                                                                                                // (forward)
                            .withVelocityY(joystick.getRawAxis(1) * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive counterclockwise
                                                                                            // with negative X (left)
            );
        } else {
            drivetrainSubsys.setDefaultCommand(
                    drivetrainSubsys.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                            .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                              // with negative X (left)
                    ));
        }

        algaeSubsys.setDefaultCommand(algaeSubsys.holdAngle());
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
            // xboxController.a().whileTrue(drivetrainSubsys.applyRequest(() -> brake));
            // xboxController.b().whileTrue(drivetrainSubsys.applyRequest(() ->
            // point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(),
            // -xboxController.getLeftX()))
            // ));

             xboxController.povRight().onTrue(algaeSubsys.intakeForward()).onFalse(algaeSubsys.intakeStop());
             xboxController.povLeft().onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
             xboxController.povDown().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae().unless(()->(algaeSubsys.coasting)));

             xboxController.a().onTrue(algaeSubsys.setAngle(-45.0));
             xboxController.b().onTrue(algaeSubsys.stopPivot());
             xboxController.x().onTrue(algaeSubsys.setAngle(-90));
             xboxController.y().onTrue(algaeSubsys.setAngle(0.0));

             xboxController2.povUp().whileTrue(superStructure.outputTopCoral());

             xboxController2.povDown().whileTrue(superStructure.intakeCoral()).onFalse(coralSubsys.stop());
             xboxController2.povLeft().whileTrue(superStructure.outputCoral());
             xboxController2.povRight().whileTrue(superStructure.outputCoralTrough());

            xboxController2.a().onTrue(elevatorSubsys.setHeight(8.0));
            xboxController2.b().onTrue(elevatorSubsys.setHeight(23.0));
            xboxController2.x().onTrue(elevatorSubsys.setHeight(0.0));
            xboxController2.y().onTrue(elevatorSubsys.setHeight(49.5));


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
        // vision.reset();
    }

    public void updateSimulation() {
        // vision.update(getPose());
    }

}
