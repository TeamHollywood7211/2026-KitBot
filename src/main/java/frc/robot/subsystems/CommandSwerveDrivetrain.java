package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // PID controller for turning to a map coordinate
    private final PIDController mapHeadingPID = new PIDController(4.0, 0, 0);

    // *UPDATE THESE WITH OFFICIAL 2026 FIELD COORDINATES FOR THE HUB*
    // Currently set to generic center-field values as placeholders
    private final Translation2d BLUE_HUB = new Translation2d(8.27, 4.10); 
    private final Translation2d RED_HUB = new Translation2d(8.27, 4.10);

    public CommandSwerveDrivetrain(com.ctre.phoenix6.swerve.SwerveDrivetrainConstants driveConstants, com.ctre.phoenix6.swerve.SwerveModuleConstants... modules) {
        super(driveConstants, modules);
        this.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
        
        // Configure PID to be continuous (0 and 360 degrees are the same)
        mapHeadingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

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

    public void updatePoseWithLimelight() {
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

    @Override
    public void periodic() {
        updatePoseWithLimelight();
        SmartDashboard.putNumber("Drivetrain/Pose X", getStatePose().getX());
        SmartDashboard.putNumber("Drivetrain/Pose Y", getStatePose().getY());
        SmartDashboard.putBoolean("Drivetrain/Aligned to Target", isAligned());
    }
    
    public boolean isAligned() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        // If we see the target, use tx. If not, assume unaligned.
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
}