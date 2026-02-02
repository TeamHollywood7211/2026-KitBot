package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    // Hardware
    private final TalonFX climberLeft = new TalonFX(ClimberConstants.kClimberLeftId);
    private final TalonFX climberRight = new TalonFX(ClimberConstants.kClimberRightId);

    // Control Requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final Follower followerRequest = new Follower(ClimberConstants.kClimberLeftId, MotorAlignmentValue.Opposed);
    private final NeutralOut brakeRequest = new NeutralOut();

    public ClimberSubsystem() {
        // Configure LEADER (Left) ---
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        
        // PID & Basic Settings
        leaderConfigs.Slot0.kP = ClimberConstants.kClimberkP;
        leaderConfigs.Slot0.kS = ClimberConstants.kClimberkS;
        leaderConfigs.Slot0.kV = ClimberConstants.kClimberkV;
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        leaderConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;

        // SOFT LIMITS (Software Stops) ***
        // Forward Limit = The "Top" (Extension)
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        
        // Reverse Limit = The "Bottom" (Retraction)
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        // Apply to Left
        climberLeft.getConfigurator().apply(leaderConfigs);

        // Configure FOLLOWER (Right) ---
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        followerConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;
        
        // Apply Soft Limits to Right as well (Safety Redundancy)
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        // Apply to Right
        climberRight.getConfigurator().apply(followerConfigs);
        
        // Start by assuming we are at 0 (Retracted)
        zeroSensors();
        
        // Default to Follower Mode
        climberRight.setControl(followerRequest);
    }

    /**
     * Resets the internal encoders to 0.
     * Call this when the hooks are fully retracted (manually or physically).
     */
    public void zeroSensors() {
        climberLeft.setPosition(0);
        climberRight.setPosition(0);
    }

    // --- Control Modes ---

    public void runClimber(double rps) {
        climberLeft.setControl(velocityRequest.withVelocity(rps));
        climberRight.setControl(followerRequest);
    }

    public void runLeftManual(double rps) {
        climberLeft.setControl(velocityRequest.withVelocity(rps));
        climberRight.setControl(brakeRequest); 
    }

    public void runRightManual(double rps) {
        climberRight.setControl(velocityRequest.withVelocity(rps));
        climberLeft.setControl(brakeRequest);
    }

    public void stop() {
        climberLeft.setControl(brakeRequest);
        climberRight.setControl(brakeRequest); 
    }
    
    // Read height for dashboard
    public double getLeftHeight() { return climberLeft.getPosition().getValueAsDouble(); }
    public double getRightHeight() { return climberRight.getPosition().getValueAsDouble(); }

    @Override
    public void periodic() {
        // You can push height to SmartDashboard here if you want
    }
}