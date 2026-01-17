package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Custom Imports for PathPlanner and Vision
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 *
 * Merged: CTRE Generator + Custom Team 7211 Logic
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // ---------------------------------------------------------
    // CUSTOM VARIABLES (Team 7211)
    // ---------------------------------------------------------
    // PID controller for turning to a map coordinate
    private final PIDController mapHeadingPID = new PIDController(4.0, 0, 0);

    // Hub Coordinates (Relative to Blue Wall) - Update with real field data if needed
    private final Translation2d BLUE_HUB = new Translation2d(8.27, 4.10); 
    private final Translation2d RED_HUB = new Translation2d(8.27, 4.10);
    // ---------------------------------------------------------

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routines */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        }, null, this)
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        commonInit();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        commonInit();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        commonInit();
    }

    /**
     * Shared initialization logic for Team 7211 Custom Features
     */
    private void commonInit() {
        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        // Custom: Trust Vision less than Odometry by default (0.1, 0.1, 0.1)
        this.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
        
        // Custom: Configure Aiming PID
        mapHeadingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // ---------------------------------------------------------
    // PATHPLANNER CONFIGURATION
    // ---------------------------------------------------------
    public void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getStatePose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
        }
    }

    public Pose2d getStatePose() {
        return getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    // ---------------------------------------------------------
    // LIMELIGHT & AIMING LOGIC
    // ---------------------------------------------------------
    public void updatePoseWithLimelight() {
        // Only run this if we are in Pipeline 0 (Localization)
        double currentPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
        if (currentPipeline != 0) return; 

        double[] poseArray = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[7]);

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (tv == 1 && poseArray.length == 7) {
            Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
            double timestamp = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds() - (poseArray[6] / 1000.0));
            this.addVisionMeasurement(visionPose, timestamp);
            SmartDashboard.putBoolean("Drivetrain/Vision Lock", true);
        } else {
            SmartDashboard.putBoolean("Drivetrain/Vision Lock", false);
        }
    }

    public void resetPoseToLimelight() {
        double[] poseArray = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[7]);

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (tv == 1 && poseArray.length == 7) {
            Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
            this.resetPose(visionPose);
        } else {
            DriverStation.reportWarning("Reset Pose Failed: No Limelight Target!", false);
        }
    }

    public void setLimelightLED(boolean on) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public void setLimelightPipeline(int pipelineIndex) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineIndex);
    }

    public boolean isAligned() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        return tv == 1 && Math.abs(tx) < 1.5;
    }

    /**
     * Hybrid Aiming:
     * 1. If TV=1 (Target Visible): Use Limelight 'tx' for precision.
     * 2. If TV=0 (Blind): Use Odometry to turn towards the Alliance HUB.
     */
    public double getAutoHeadingRate() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (tv == 1) {
            // Precision Mode: P-controller on Limelight offset
            return tx * 0.05; 
        } else {
            // Rough Align Mode: Calculate angle to Hub using Odometry
            Translation2d targetLocation = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
                == DriverStation.Alliance.Blue ? BLUE_HUB : RED_HUB;
            
            double targetAngleRadians = Math.atan2(
                targetLocation.getY() - getStatePose().getY(),
                targetLocation.getX() - getStatePose().getX()
            );

            // Use PID to turn to that map angle
            return mapHeadingPID.calculate(getStatePose().getRotation().getRadians(), targetAngleRadians);
        }
    }

    // ---------------------------------------------------------
    // STANDARD CTRE PERIODIC & SYSID
    // ---------------------------------------------------------
    
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        // 1. CTRE Logic: Handle Alliance Perspective for Field-Centric Driving
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

        // 2. Custom Logic: Update Vision and Dashboard
        updatePoseWithLimelight();
        SmartDashboard.putNumber("Drivetrain/Pose X", getStatePose().getX());
        SmartDashboard.putNumber("Drivetrain/Pose Y", getStatePose().getY());
        SmartDashboard.putBoolean("Drivetrain/Aligned to Target", isAligned());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}