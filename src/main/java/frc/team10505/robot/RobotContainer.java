/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import static edu.wpi.first.units.Units.*;
import static frc.team10505.robot.Constants.OperatorInterfaceConstants.*;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // was .75 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator interfaces */
    // private final CommandXboxController xboxController = new
    // CommandXboxController(kXboxControllerPort);
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

    private final Vision vision = new Vision();
    /* Superstructure */
    private final Superstructure superStructure = new Superstructure(coralSubsys, algaeSubsys, elevatorSubsys,
            drivetrainSubsys, vision);

    /* Autonomous */
    private final SendableChooser<Command> autoChooser;// = new SendableChooser<>();
    private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

    public RobotContainer() {
        NamedCommands.registerCommand("setElevToZero", elevatorSubsys.setHeight(0.0));
        NamedCommands.registerCommand("setElevToNine", elevatorSubsys.setHeight(9.0));
        NamedCommands.registerCommand("setElevTo24", elevatorSubsys.setHeight(24.0));
        NamedCommands.registerCommand("setElevTo49/5", elevatorSubsys.setHeight(48.5));

        NamedCommands.registerCommand("intakeCoral", superStructure.intakeCoral());

        NamedCommands.registerCommand("autoElevDown", superStructure.autoElevDown());
        NamedCommands.registerCommand("autoScoreCoralL3", superStructure.autoScoreCoralL3());
        NamedCommands.registerCommand("autoScoreCoralL4", superStructure.autoScoreCoralL4());
        NamedCommands.registerCommand("autoScoreCoralL2", superStructure.autoScoreCoralL2());
        NamedCommands.registerCommand("autoScoreCoralL1", superStructure.autoScoreCoralL1());
        NamedCommands.registerCommand("Test", Commands.print("isworking"));
        NamedCommands.registerCommand("autoL4Bump", superStructure.autoL4Bump());

        drivetrainSubsys.configDrivetrainSubsys();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Polarity Chooser", polarityChooser);
        polarityChooser.setDefaultOption("Default", 1.0);
        polarityChooser.addOption("positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);

        configDefaultCommands();
        configButtonBindings();
        configAutonomous();
    }

    /**
     * Function that is called in the constructor where we configure default
     * commands for the subsytems.
     */
    private void configDefaultCommands() {

        if (Utils.isSimulation()) {
            drivetrainSubsys.setDefaultCommand(
                    drivetrainSubsys.applyRequest(() -> drive
                            .withVelocityX(-joystick.getRawAxis(0) * polarityChooser.getSelected() * MaxSpeed) // Drive
                            // forward
                            // with
                            // negative
                            // Y
                            // (forward)
                            .withVelocityY(joystick.getRawAxis(1) * polarityChooser.getSelected() * MaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                            .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive counterclockwise
                                                                                            // with negative X (left)
            );

        } else {
            drivetrainSubsys.setDefaultCommand(
                    drivetrainSubsys.applyRequest(
                            () -> drive.withVelocityX(-joystick.getY() * 0.6 * polarityChooser.getSelected() * MaxSpeed) // Drive

                                    .withVelocityY(-joystick.getX() * 0.8 * polarityChooser.getSelected() * MaxSpeed) // Drive
                                                                                                                      // left
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                                    .withRotationalRate(-joystick.getTwist() * 1.6 * MaxAngularRate) // Drive
                                                                                                     // counterclockwise
                                                                                                     // with negative X
                                                                                                     // (left)
                    ));
        }

        algaeSubsys.setDefaultCommand(algaeSubsys.holdAngle());
    }

    private SwerveRequest.ApplyRobotSpeeds robotDrive = new SwerveRequest.ApplyRobotSpeeds();

    /**
     * Function that is called in the constructor where we configure operator
     * interface button bindings.
     */
    private void configButtonBindings() {

        // button bindings for simulations
        if (Utils.isSimulation()) {
            // joystick.button(1).whileTrue(elevatorSubsys.setHeight(0.0));
            joystick.button(2).whileTrue(elevatorSubsys.setHeight(1.0));
            joystick.button(3).whileTrue(elevatorSubsys.setHeight(2.5));
            joystick.button(4).whileTrue(elevatorSubsys.setHeight(4.0));

            // joystick2.button(1).whileTrue(drivetrainSubsys.applyRequest(() ->
            // robotDrive));

        } else {
            // button bindings for real life

            // automatically added bindings from ctre swerve generated project
            // KEEP to reference for alignment stuff?
            // xboxController.a().whileTrue(drivetrainSubsys.applyRequest(() -> brake));
            // xboxController.b().whileTrue(drivetrainSubsys.applyRequest(() ->
            // point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(),
            // -xboxController.getLeftX()))
            // ));

            // Old bindings for the xbox controller
            // KEEP in case we swtich back
            // xboxController.leftBumper().onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
            // xboxController.rightBumper().onTrue(algaeSubsys.intakeForward()).onFalse(superStructure.holdAlgae());
            // //xboxController.povDown().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());

            // xboxController.leftTrigger().whileTrue( drivetrainSubsys.applyRequest(() ->
            // drive.withVelocityX(-xboxController.getLeftY()* polarityChooser.getSelected()
            // * 0.2* MaxSpeed) // Drive
            // .withVelocityY(-xboxController.getLeftX() * polarityChooser.getSelected()*
            // 0.2* MaxSpeed) // Drive left with negative X (left)
            // .withRotationalRate(-xboxController.getRightX() * 0.6* MaxAngularRate)));

            // xboxController.a().onTrue(algaeSubsys.setAngle(-18));
            // xboxController.b().onTrue(algaeSubsys.stopPivot());
            // xboxController.x().onTrue(algaeSubsys.setAngle(-90));
            // xboxController.y().onTrue(algaeSubsys.setAngle(5));

            // xboxController.povDown().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());

            // NEW joystick bindings
            joystick.button(3).onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
            joystick.button(2).onTrue(algaeSubsys.intakeForward()).onFalse(superStructure.holdAlgae());

            joystick.trigger()
                    .whileTrue(drivetrainSubsys.applyRequest(
                            () -> drive.withVelocityX(-joystick.getY() * 0.2 * polarityChooser.getSelected() * MaxSpeed) // Drive
                                    .withVelocityY(-joystick.getX() * 0.2 * polarityChooser.getSelected() * MaxSpeed) // Drive left with negative X (left)
                                    .withRotationalRate(-joystick.getTwist() * 0.5 * MaxAngularRate) // Drive counterclockwise with negative X (left)
                                    ));

            joystick.button(12).onTrue(algaeSubsys.setAngle(-22));// -18
            joystick.button(9).onTrue(algaeSubsys.setAngle(-90));
            joystick.button(10).onTrue(algaeSubsys.setAngle(5));
            joystick.button(4).whileTrue(superStructure.alignToReef());// currently NON functional

            // operator bindings
            xboxController2.povUp().whileTrue(superStructure.outputTopCoral());
            xboxController2.povDown().onTrue(superStructure.intakeCoral()).onFalse(coralSubsys.stop());
            xboxController2.povLeft().whileTrue(superStructure.outputCoral());
            xboxController2.povRight().whileTrue(superStructure.outputCoralTrough());

            xboxController2.a().onTrue(elevatorSubsys.setHeight(8.0));
            xboxController2.b().onTrue(elevatorSubsys.setHeight(24.0));
            xboxController2.x().onTrue(elevatorSubsys.setHeight(0.0));
            xboxController2.y().onTrue(elevatorSubsys.setHeight(48.5));

            // automoatically added from the ctre generated swerve drive
            // probably is important?
            drivetrainSubsys.registerTelemetry(logger::telemeterize);
        }
    }

    // methods that allow us to select and use our auton selected in our dashboards
    private void configAutonomous() {
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // called periodically in robot.java, updates all our pose estimation stuff for
    // drivetrain and vision
    // TODO test vision pose estimtaion with a tag, and taking it away
    public void updatePose() {
        // puts the drivetrain pose on our dashboards
        SmartDashboard.putNumber("estimated drive pose x", drivetrainSubsys.getState().Pose.getX());
        SmartDashboard.putNumber("estimated drive pose y", drivetrainSubsys.getState().Pose.getY());
        SmartDashboard.putNumber("estimated drive pose rotation",
                drivetrainSubsys.getState().Pose.getRotation().getDegrees());

        // puts the pose from the reef cam on our dashboards if it sees a tag
        if (vision.getReefCamEstimatedPose().isPresent()) {
            SmartDashboard.putNumber("Reef Cam Pose X", vision.getReefCamEstimatedPose().get().estimatedPose.getX());
            SmartDashboard.putNumber("Reef Cam Pose Y", vision.getReefCamEstimatedPose().get().estimatedPose.getY());
            SmartDashboard.putNumber("Reef Cam Pose Angle",
                    vision.getReefCamEstimatedPose().get().estimatedPose.toPose2d().getRotation().getDegrees());
        }

        // allows us to reset our pose
        // TODO take out for matches
        // for auton testing at LSSU, change the pose to the starting pose we have in
        // pathplanner. Note that the rotation2d value is found in the starting state,
        // NOT the start pose
        if (joystick.button(5).getAsBoolean()) {
            drivetrainSubsys.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        }

        // //stuff from ctre example, I dont think its needed(took it out morning of
        // 3/14)
        // var driveState = drivetrainSubsys.getState();
        // double headingDeg = driveState.Pose.getRotation().getDegrees();
        // double omegaRps =
        // Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        // ***I think having this here gives a pose of 0,0,0 */
        // *** therefore, i commented it out (morning of 3/14)*/
        // var mostRecentReefCamPose = new Pose2d();

        // if the camera has a tag in sight, it will calculate a pose and add to the
        // drivetrain
        // if it fails, it will print the message
        try {
            var reefCamPose = vision.getReefCamEstimatedPose().get().estimatedPose.toPose2d();
            var mostRecentReefCamPose = reefCamPose;
            drivetrainSubsys.addVisionMeasurement(mostRecentReefCamPose, vision.lastReefCamEstimateTimestamp);
        } catch (Exception e) {
            Commands.print("reef cam pose failed");
        }

        // ***If the stuff above doesn't work(i changed it 3/14 in the morning), try
        // adding these */
        // var newestPose = drivetrainSubsys.getState().Pose;
        // drivetrainSubsys.resetPose(newestPose);
    }

    // method called periodically in robot.java only when we simulate the code
    // makes the robot icon appear in the correct place
    public void updateSimulation() {
        vision.visionSim.update(drivetrainSubsys.getState().Pose);
        vision.visionSim.getDebugField();
    }

    // method called once in robot.java to reset our simulated field
    public void resetSimulation() {
        vision.reset();
    }

}
