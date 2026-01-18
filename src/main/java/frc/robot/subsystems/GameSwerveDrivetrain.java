package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d; // <--- FOR DASHBOARD

public class GameSwerveDrivetrain extends CommandSwerveDrivetrain {

    private final PIDController mapHeadingPID = new PIDController(4.0, 0, 0);
    private final Translation2d BLUE_HUB = new Translation2d(8.27, 4.10);
    private final Translation2d RED_HUB = new Translation2d(8.27, 4.10);
    
    // Field Object for Elastic
    private final Field2d field = new Field2d();

    public GameSwerveDrivetrain(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveConstants, modules);
        configureCustomSettings();
        // Put field on dashboard
        SmartDashboard.putData("Field", field);
    }

    private void configureCustomSettings() {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
        mapHeadingPID.enableContinuousInput(-Math.PI, Math.PI);
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

    public Pose2d getStatePose() { return getState().Pose; }
    public ChassisSpeeds getRobotRelativeSpeeds() { return getState().Speeds; }

    public void resetPoseToLimelight() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0);
        double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

        if (tv == 1 && poseArray.length >= 7) {
            Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
            this.resetPose(visionPose);
        } else {
            DriverStation.reportWarning("Reset Pose Failed: No Limelight Target!", false);
        }
    }

    public void seedFieldCentric() {
        // Safe Reset: Keep X/Y, reset Rotation to 0
        Pose2d currentPose = this.getState().Pose;
        this.resetPose(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
    }

    public void updatePoseWithLimelight() {
        if (NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").getDouble(0) != 0) return;
        updatePoseFromCamera("limelight-front");
        updatePoseFromCamera("limelight-rear");
    }

    private void updatePoseFromCamera(String cameraName) {
        double[] poseArray = NetworkTableInstance.getDefault().getTable(cameraName).getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        double tv = NetworkTableInstance.getDefault().getTable(cameraName).getEntry("tv").getDouble(0);

        if (tv == 1 && poseArray.length == 7) {
            Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
            double timestamp = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds() - (poseArray[6] / 1000.0));
            this.addVisionMeasurement(visionPose, timestamp);
        }
    }

    public void setLimelightPipeline(int pipelineIndex) {
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setNumber(pipelineIndex);
        NetworkTableInstance.getDefault().getTable("limelight-rear").getEntry("pipeline").setNumber(pipelineIndex);
    }

    public void setLimelightLED(boolean on) {
        int mode = on ? 3 : 1;
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ledMode").setNumber(mode);
        NetworkTableInstance.getDefault().getTable("limelight-rear").getEntry("ledMode").setNumber(mode);
    }

    public boolean isAligned() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0);
        return tv == 1 && Math.abs(tx) < 1.5;
    }

    public double getAutoHeadingRate() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0);

        if (tv == 1) {
            return tx * 0.05; 
        } else {
            Translation2d targetLocation = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
                == DriverStation.Alliance.Blue ? BLUE_HUB : RED_HUB;
            
            double targetAngleRadians = Math.atan2(
                targetLocation.getY() - getStatePose().getY(),
                targetLocation.getX() - getStatePose().getX()
            );
            return mapHeadingPID.calculate(getStatePose().getRotation().getRadians(), targetAngleRadians);
        }
    }

    @Override
    public void periodic() {
        super.periodic(); 
        updatePoseWithLimelight();
        // Update Elastic Field
        field.setRobotPose(getStatePose());
        SmartDashboard.putBoolean("Drivetrain/Aligned", isAligned());
    }
}