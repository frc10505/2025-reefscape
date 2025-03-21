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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

import org.ejml.data.Matrix;
import org.ejml.equation.MatrixConstructor;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // was .75 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private SwerveRequest.ApplyRobotSpeeds robotDrive = new SwerveRequest.ApplyRobotSpeeds();

    /* Operator interfaces */
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final CommandXboxController xboxController2 = new CommandXboxController(kControlPanelPort);

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final CoralSubsystem coralSubsys = new CoralSubsystem();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();

    /* Superstructure */
    private final Superstructure superStructure = new Superstructure(coralSubsys, algaeSubsys, elevatorSubsys,
            drivetrainSubsys);

    /* Autonomous */
    private final SendableChooser<Command> autoChooser;// = new SendableChooser<>();
    private final SendableChooser<Double> polarityChooser = new SendableChooser<>();

    public RobotContainer() {

        NamedCommands.registerCommand("Test", Commands.print("auto command stuff is working"));

        NamedCommands.registerCommand("setElevToZero", elevatorSubsys.setHeight(0.0));
        NamedCommands.registerCommand("setElevToNine", elevatorSubsys.setHeight(9.0));
        NamedCommands.registerCommand("setElevTo24", elevatorSubsys.setHeight(24.0));
        NamedCommands.registerCommand("setElevTo49/5", elevatorSubsys.setHeight(48.5));

        NamedCommands.registerCommand("intakeCoral", superStructure.intakeCoral());

        NamedCommands.registerCommand("autoScoreCoralL3", superStructure.autoScoreCoralL3());
        NamedCommands.registerCommand("autoScoreCoralL4", superStructure.autoScoreCoralL4());
        NamedCommands.registerCommand("autoScoreCoralL2", superStructure.autoScoreCoralL2());
        NamedCommands.registerCommand("autoScoreCoralL1", superStructure.autoScoreCoralL1());

        NamedCommands.registerCommand("autoElevDown", superStructure.autoElevDown());
        NamedCommands.registerCommand("autoL4Bump", superStructure.autoL4Bump());

        NamedCommands.registerCommand("setPose_FarRightReef_LeftSide", superStructure.setPose(4.973, 2.861, 120));
        NamedCommands.registerCommand("setPose_FarRightReef_RightSide", superStructure.setPose(5.257, 3.026, 120));
        NamedCommands.registerCommand("setPose_FarMiddleReef_LeftSide", superStructure.setPose(5.74, 3.862, 180));
        NamedCommands.registerCommand("setPose_FarMiddleReef_RightSide", superStructure.setPose(5.74, 4.19, 180));

        NamedCommands.registerCommand("autoAlignLeft", superStructure.autoAlignLeft());
        NamedCommands.registerCommand("autoAlignRight", superStructure.autoAlignRight());
        NamedCommands.registerCommand("autoDriveForward", superStructure.autoDriveForward());
        NamedCommands.registerCommand("autoDriveForwardTillSeesRight", superStructure.autoDriveForwardTillSeesRight());


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
            drivetrainSubsys.setDefaultCommand(drivetrainSubsys.applyRequest(() -> drive
                    .withVelocityX(-xboxController.getLeftY() * polarityChooser.getSelected()
                            * 0.8 * MaxSpeed) // Drive
                    .withVelocityY(-xboxController.getLeftX() * polarityChooser.getSelected() *
                            0.8 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxController.getRightX() * 3.2 * MaxAngularRate))); //2.5
            // drivetrainSubsys.applyRequest(
            // () -> drive.withVelocityX(-joystick.getY() * 0.8 *
            // polarityChooser.getSelected() * MaxSpeed) //0.6// Drive

            // .withVelocityY(-joystick.getX() * 0.8 * polarityChooser.getSelected() *
            // MaxSpeed) // Drive
            // // left
            // // with
            // // negative
            // // X
            // // (left)
            // //.withRotationalRate(-joystick.getTwist() * 1.6 * MaxAngularRate) // Drive
            // // counterclockwise
            // // with negative X
            // // (left)
            // ));
        }

        algaeSubsys.setDefaultCommand(algaeSubsys.holdAngle());
    }

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
            xboxController.leftBumper().onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
            xboxController.rightBumper().onTrue(algaeSubsys.intakeForward()).onFalse(superStructure.holdAlgae());
            //xboxController.povDown().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());

            xboxController.leftTrigger()
                    .whileTrue(drivetrainSubsys.applyRequest(() -> drive
                            .withVelocityX(-xboxController.getLeftY() * polarityChooser.getSelected()
                                    * 0.2 * MaxSpeed) // Drive
                            .withVelocityY(-xboxController.getLeftX() * polarityChooser.getSelected() *
                                    0.2 * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-xboxController.getRightX() * 0.6 * MaxAngularRate)));

            xboxController.a().onTrue(algaeSubsys.setAngle(-18));
            xboxController.b().onTrue(resetPose());//onTrue(algaeSubsys.stopPivot());
            xboxController.x().onTrue(algaeSubsys.setAngle(-90));
            xboxController.y().onTrue(algaeSubsys.setAngle(5));

            xboxController.start().onTrue(resetGyro());

           // xboxController.l().onTrue(superStructure.grabAlgae()).onFalse(superStructure.holdAlgae());
            xboxController.povLeft().whileTrue(//superStructure.autoAlignLeft());
                    drivetrainSubsys.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, 0.3, 0.0))).until(() -> !drivetrainSubsys.seesLeftSensor()));
            xboxController.povRight().whileTrue(//superStructure.autoAlignRight());
                      drivetrainSubsys.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.0, -0.4, 0.0))).until(() -> !drivetrainSubsys.seesRightSensor()));//.3
            // xboxController.povUp().whileTrue(superStructure.autoTwist());//.until(() -> (drivetrainSubsys.seesLeftSensor() && drivetrainSubsys.seesRightSensor())));
            //         //   drivetrainSubsys.applyRequest(() -> robotDrive.withSpeeds(new ChassisSpeeds(0.3, 0.0, 0.0))).until(() -> (drivetrainSubsys.seesRightSensor() && drivetrainSubsys.seesLeftSensor())));
    
            // NEW joystick bindings

            // joystick.trigger()
            // .whileTrue(drivetrainSubsys.applyRequest(
            // () -> drive.withVelocityX(-joystick.getY() * 0.2 *
            // polarityChooser.getSelected() * MaxSpeed) // Drive
            // .withVelocityY(-joystick.getX() * 0.2 * polarityChooser.getSelected() *
            // MaxSpeed) // Drive
            // // left
            // // with
            // // negative
            // // X
            // // (left)
            // .withRotationalRate(-joystick.getTwist() * 0.5 * MaxAngularRate) // Drive
            // // counterclockwise
            // // with negative X
            // // (left)
            // ));

            // joystick.button(12).onTrue(algaeSubsys.setAngle(-22));// -18
            // joystick.button(9).onTrue(algaeSubsys.setAngle(90));//-90
            // joystick.button(10).onTrue(algaeSubsys.setAngle(5));

            // joystick.button(3).onTrue(algaeSubsys.intakeReverse()).onFalse(algaeSubsys.intakeStop());
            // joystick.button(5).onTrue(algaeSubsys.intakeForward()).onFalse(superStructure.holdAlgae());

            // joystick.povLeft().whileTrue(drivetrainSubsys.alignLeft());

            // operator bindings
            xboxController2.povUp().whileTrue(superStructure.outputTopCoral());
            xboxController2.povDown().onTrue(superStructure.intakeCoral()).onFalse(coralSubsys.stop());
            xboxController2.povLeft().whileTrue(superStructure.outputCoral());
            xboxController2.povRight().whileTrue(superStructure.outputCoralTrough());

            xboxController2.a().onTrue(elevatorSubsys.setHeight(8.0));
            xboxController2.b().onTrue(elevatorSubsys.setHeight(23.0));
            xboxController2.x().onTrue(elevatorSubsys.setHeight(0.0));
            xboxController2.y().onTrue(elevatorSubsys.setHeight(48.5));
            xboxController2.rightBumper().onTrue(superStructure.manualL4Bump());

            // automoatically added from the ctre generated swerve drive
            // probably is important?
            drivetrainSubsys.registerTelemetry(logger::telemeterize);
        }
    }

    private Command resetGyro(){
        return Commands.runOnce(() -> {
            drivetrainSubsys.getPigeon2().reset();;
            });
    }


    private Command resetPose(){
        return Commands.runOnce(()-> {
            drivetrainSubsys.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        });
    }
    // methods that allow us to select and use our auton selected in our dashboards
    private void configAutonomous() {
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateDriveSensors() {
        SmartDashboard.putBoolean("left drive sensor", drivetrainSubsys.seesLeftSensor());
        SmartDashboard.putBoolean("right drive sensor", drivetrainSubsys.seesRightSensor());
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
        SmartDashboard.putNumber("Gyro rot", Math.abs(drivetrainSubsys.getPigeon2().getRotation2d().getDegrees() % 60));

        // puts the pose from the reef cam on our dashboards if it sees a tag

        // allows us to reset our pose
        // TODO take out for matches
        // for auton testing at LSSU, change the pose to the starting pose we have in
        // pathplanner. Note that the rotation2d value is found in the starting state,
        // NOT the start pose
        // if (joystick.button(5).getAsBoolean()) {
        //     drivetrainSubsys.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        // }

        // // ***I think having this here gives a pose of 0,0,0 */
        // // *** therefore, i commented it out (morning of 3/14)*/
        // var mostRecentReefCamPose = new Pose2d();

        // // if the camera has a tag in sight, it will calculate a pose and add to the
        // // drivetrain
        // // if it fails, it will print the message

        // try {

        //     var visionEst = vision.getReefCamEstimatedPose();
        //     visionEst.ifPresent(est -> {
        //         drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        //         SmartDashboard.putNumber("reef cam pose x", est.estimatedPose.toPose2d().getX());
        //         SmartDashboard.putNumber("reef cam pose y", est.estimatedPose.toPose2d().getY());
        //         SmartDashboard.putNumber("reef cam pose rot", est.estimatedPose.toPose2d().getRotation().getDegrees());
        //     });

        //     // var reefCamPose =
        //     // vision.getReefCamEstimatedPose().get().estimatedPose.toPose2d();
        //     // mostRecentReefCamPose = reefCamPose;
        //     // // double [][] array = {{3}, {4}, {3}};
        //     // //Nat<N3> x = new Nat<N3>();
        //     // drivetrainSubsys.addVisionMeasurement(mostRecentReefCamPose,
        //     // vision.lastReefCamEstimateTimestamp);

        //     // SmartDashboard.putNumber("reef cam pose x", mostRecentReefCamPose.getX());
        //     // SmartDashboard.putNumber("reef cam pose y", mostRecentReefCamPose.getY());
        //     // SmartDashboard.putNumber("reef cam pose rot",
        //     // mostRecentReefCamPose.getRotation().getDegrees());

        // } catch (Exception e) {
        //     Commands.print("reef cam pose failed");
        // }

        // ***If the stuff above doesn't work(i changed it 3/14 in the morning), try
        // adding these */
        // var newestPose = drivetrainSubsys.getState().Pose;
        // drivetrainSubsys.resetPose(newestPose);
    }



    // method called periodically in robot.java only when we simulate the code
    // makes the robot icon appear in the correct place
    // public void updateSimulation() {
    //     vision.visionSim.update(drivetrainSubsys.getState().Pose);
    //     vision.visionSim.getDebugField();
    // }

    // // method called once in robot.java to reset our simulated field
    // public void resetSimulation() {
    //     vision.reset();
    // }

}
