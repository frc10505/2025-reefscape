package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team10505.robot.Vision;
import frc.team10505.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import static frc.team10505.robot.Constants.DrivetrainConstants.*;


public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {

private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
//SwerveRequest.ApplyChassisSpeeds();

private Vision vision = new Vision();


double turnDistance = 0;
double strafeDistance = 0;
double skewDistance = 0;

private final PIDController strafeController = new PIDController(kStrafeP, kStrafeI, kStrafeD);
private final PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
private final PIDController distanceController = new PIDController(kDistanceP, kDistanceI, kDistanceD);

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;



//Three constructors that use different parameters
    public DrivetrainSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
     //The command we referense to make the drivetrain move
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }


        // PhotonPipelineResult result = vision.reefCam.getLatestResult();
        // var alliance = DriverStation.getAlliance();
        // if(result.hasTargets()){
        //     SmartDashboard.putNumber("Best target id", result.getBestTarget().fiducialId);
        //     //if(alliance.get() == Alliance.Red){
        //   if((result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 6)|(result.getBestTarget().fiducialId == 7)|(result.getBestTarget().fiducialId == 8)|(result.getBestTarget().fiducialId == 9)|(result.getBestTarget().fiducialId == 11)|(result.getBestTarget().fiducialId == 17)|(result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 19)|(result.getBestTarget().fiducialId == 20)|(result.getBestTarget().fiducialId == 21)|(result.getBestTarget().fiducialId == 22)){//(6|7|8|9|10|11|17|18|19|20|21|22)){
                
        //     if(MathUtil.isNear(3.0, vision.reefCam.getLatestResult().getBestTarget().getYaw(),  5)){
        //         strafeDistance = 0;
        //     }else{
        //          strafeDistance = strafeController.calculate(result.getBestTarget().getYaw(), kLeftYawSetpoint); 
        //         SmartDashboard.putNumber("red strafe distance", strafeDistance);
        //     }
        // }}
        // if(result.hasTargets()){
        //     SmartDashboard.putNumber("Best target id", result.getBestTarget().fiducialId);
        //     //if(alliance.get() == Alliance.Red){
        //   if((result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 6)|(result.getBestTarget().fiducialId == 7)|(result.getBestTarget().fiducialId == 8)|(result.getBestTarget().fiducialId == 9)|(result.getBestTarget().fiducialId == 11)|(result.getBestTarget().fiducialId == 17)|(result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 19)|(result.getBestTarget().fiducialId == 20)|(result.getBestTarget().fiducialId == 21)|(result.getBestTarget().fiducialId == 22)){//(6|7|8|9|10|11|17|18|19|20|21|22)){
                
        //     if(MathUtil.isNear(3.0, vision.reefCam.getLatestResult().getBestTarget().getPitch(),  10)){
        //         turnDistance = 0;
        //     }else{
        //          turnDistance = strafeController.calculate(result.getBestTarget().getPitch(), 180.0); 
        //         SmartDashboard.putNumber("red turn distance", turnDistance);
        //     }
        // }}

        // if(result.hasTargets()){
        //     SmartDashboard.putNumber("Best target id", result.getBestTarget().fiducialId);
        //     //if(alliance.get() == Alliance.Red){
        //   if((result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 6)|(result.getBestTarget().fiducialId == 7)|(result.getBestTarget().fiducialId == 8)|(result.getBestTarget().fiducialId == 9)|(result.getBestTarget().fiducialId == 11)|(result.getBestTarget().fiducialId == 17)|(result.getBestTarget().fiducialId == 18)|(result.getBestTarget().fiducialId == 19)|(result.getBestTarget().fiducialId == 20)|(result.getBestTarget().fiducialId == 21)|(result.getBestTarget().fiducialId == 22)){//(6|7|8|9|10|11|17|18|19|20|21|22)){
                
        //     if(MathUtil.isNear(3.0, vision.reefCam.getLatestResult().getBestTarget().getSkew(),  5)){
        //         skewDistance = 0;
        //     }else{
        //          skewDistance = strafeController.calculate(result.getBestTarget().getSkew(), kLeftDistanceSetpoint); 
        //         SmartDashboard.putNumber("red skew distance", skewDistance);
        //     }
        // }}

        // if(runDeathButton){
        //     this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(skewDistance, strafeDistance, turnDistance)));    
        // }

       
        //  SmartDashboard.putBoolean("sees tag", vision.reefCam.getLatestResult().hasTargets());
        //  SmartDashboard.putBoolean("run Death Button", runDeathButton);
        // // SmartDashboard.putBoolean("near yaw", isNearYaw());
        // // SmartDashboard.putBoolean("near turn", isNearTurn());
        // // SmartDashboard.putBoolean("near skew", isNearSkew());
        // // SmartDashboard.putBoolean("near target", isNearTarget());


    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


//DEATH BUTTON 2.0
//BE AFRAID

private boolean runDeathButton = false;
public Command alignWithReef() {
    return runEnd(() -> {
        runDeathButton = true;
        // this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(skewDistance, strafeDistance, turnDistance)));    
    }, () -> {
        runDeathButton = false;
    }
    );
}







// public boolean isNearYaw() {
//     return MathUtil.isNear(3.0, vision.reefCam.getLatestResult().getBestTarget().getYaw(),  2);
// }

// public boolean isNearTurn() {
//     return MathUtil.isNear(180.0, vision.reefCam.getLatestResult().getBestTarget().getPitch(),  2);
// }

// public boolean isNearSkew() {
//     return MathUtil.isNear(3.0, vision.reefCam.getLatestResult().getBestTarget().getSkew(),  2);
// }

       
// public boolean isNearTarget() {
//     return (isNearSkew() & isNearTurn() & isNearYaw());
// }


    public void configDrivetrainSubsys() {
        try {
            var config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("something may or may not be broken, idk", ex.getStackTrace());
        }
    }








//     public final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

//   private final PhotonCamera photonCamera = new PhotonCamera("reefCam");
//   //private final PhotonCamera photonNote = new PhotonCamera(kNoteCameraName);
//   private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
//       PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToMod0CameraTransform);

//   private double lastEstimateTimestamp = 0.0;

// //   public Vision() {
// //     configVision();
// //   }

//   public PhotonPipelineResult getLatestResult() {
//     return photonCamera.getLatestResult();
//   }

// //   public PhotonPipelineResult getNoteResult() {
// //     return photonNote.getLatestResult();
// //   }

//   public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
//     var visionEstimate = photonPoseEstimator.update(photonCamera.getLatestResult());
//     double latestTimestamp = getLatestResult().getTimestampSeconds();
//     boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;

//     if (newResult) {
//       lastEstimateTimestamp = latestTimestamp;
//     }

//     return visionEstimate;
//   }

//   public double getTargetHeight() {
//     double targetHeight;
    
//     var result = getLatestResult();

//     if (result.hasTargets()) {
//       targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
//     } else {
//       targetHeight = 0.0;
//     }

//     return targetHeight;
//   }

// //   public double getTargetDistance() {
// //     double targetDistance;

// //     var result = getLatestResult();

// //     if (result.hasTargets()) {
// //       targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
// //           kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
// //     } else {
// //       targetDistance = 0.0;
// //     }

// //     return targetDistance;
// //   }




    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // same thing as before, but could be used in place of it if we use the standard deviation of vision measurments(I have no idea how to do that!)
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        // Standard deviations of the vision measurement for multi tag.
        // X position in meters, y position in meters, and heading in radians.
        Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), multiTagStdDevs);
    }
}
