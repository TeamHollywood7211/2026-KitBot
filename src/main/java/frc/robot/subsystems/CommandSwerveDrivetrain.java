package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; 
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final PIDController headingPID = new PIDController(0.05, 0, 0);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        headingPID.setTolerance(1.0);
        if (Utils.isSimulation()) startSimThread();
    }

    public double getAutoHeadingRate() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        
        if (tv < 1) return 0;
        return headingPID.calculate(tx, 0);
    }

    public void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.getState().Pose,      
                this::resetPose,                
                this::getRobotRelativeSpeeds,    
                this::driveRobotRelative,       
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0)  
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed", e.getStackTrace());
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public void updatePoseWithLimelight() {
        double[] poseArray = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[7]);

        if (poseArray.length > 0 && NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
            addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds() - (poseArray[6] / 1000.0)));
            SmartDashboard.putBoolean("Drivetrain/Vision Lock", true);
        } else {
            SmartDashboard.putBoolean("Drivetrain/Vision Lock", false);
        }
    }

    @Override
    public void periodic() {
        updatePoseWithLimelight();
        
        ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
        double linearVelocity = Math.sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

        SmartDashboard.putNumber("Drivetrain/Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Drivetrain/Speed mps", linearVelocity);
        SmartDashboard.putNumber("Drivetrain/Heading Degrees", getState().Pose.getRotation().getDegrees());
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            updateSimState(currentTime - m_lastSimTime, RobotController.getBatteryVoltage());
            m_lastSimTime = currentTime;
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return run(() -> {}); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return run(() -> {}); }
}