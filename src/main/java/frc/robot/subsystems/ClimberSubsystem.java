package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut; // Raw Power Control
import com.ctre.phoenix6.controls.MotionMagicVoltage; // Smooth Auto Control
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberLeft = new TalonFX(ClimberConstants.kClimberLeftId);
    private final TalonFX climberRight = new TalonFX(ClimberConstants.kClimberRightId);

    // CONTROLS
    // Manual Request: -1.0 to 1.0 (Raw Percent Output)
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);
    
    // Auto Request: Motion Magic (Smooth Position Control)
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    
    private final NeutralOut brakeRequest = new NeutralOut();

    // FOLLOWER SETUP
    private final Follower followerRequest = new Follower(
        ClimberConstants.kClimberLeftId, 
        ClimberConstants.kFollowerAlignment
    );

    public ClimberSubsystem() {
        // --- 1. Configure LEADER (Left) ---
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        
        // PID & Motion Magic
        leaderConfigs.Slot0.kP = ClimberConstants.kClimberkP;
        leaderConfigs.Slot0.kS = ClimberConstants.kClimberkS;
        leaderConfigs.Slot0.kV = ClimberConstants.kClimberkV;
        leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.kMotionMagicCruiseVelocity;
        leaderConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.kMotionMagicAcceleration;
        leaderConfigs.MotionMagic.MotionMagicJerk = ClimberConstants.kMotionMagicJerk;

        // Power & Safety
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        leaderConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;

        // Hardware Direction
        leaderConfigs.MotorOutput.Inverted = ClimberConstants.kLeftInverted;

        // Soft Limits
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        climberLeft.getConfigurator().apply(leaderConfigs);

        // --- 2. Configure FOLLOWER (Right) ---
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        
        followerConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        followerConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;
        
        // Hardware Direction
        followerConfigs.MotorOutput.Inverted = ClimberConstants.kRightInverted;
        
        // Redundant Soft Limits
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        climberRight.getConfigurator().apply(followerConfigs);
        
        zeroSensors();
        
        // Start following
        climberRight.setControl(followerRequest);
    }

    public void zeroSensors() {
        climberLeft.setPosition(0);
        climberRight.setPosition(0);
    }

    // --- AUTO COMMANDS (Motion Magic) ---

    public void extendMax() {
        moveToPosition(ClimberConstants.kMaxHeightRotations);
    }

    public void retractToClimb() {
        moveToPosition(ClimberConstants.kClimbTargetHeight);
    }

    private void moveToPosition(double targetRotations) {
        // Clamp target
        double safeTarget = Math.max(targetRotations, ClimberConstants.kMinHeightRotations);
        safeTarget = Math.min(safeTarget, ClimberConstants.kMaxHeightRotations);
        
        climberLeft.setControl(positionRequest.withPosition(safeTarget));
        climberRight.setControl(followerRequest);
    }

    // --- MANUAL CONTROLS (Raw Percent Output) ---

    /**
     * Standard Drive: Left drives, Right follows.
     * @param percent Output from -1.0 to 1.0
     */
    public void runClimber(double percent) {
        climberLeft.setControl(manualRequest.withOutput(percent));
        climberRight.setControl(followerRequest);
    }

    /**
     * Manual Left Override.
     * @param percent Output from -1.0 to 1.0
     */
    public void runLeftManual(double percent) {
        climberLeft.setControl(manualRequest.withOutput(percent));
        climberRight.setControl(brakeRequest);
    }

    /**
     * Manual Right Override.
     * @param percent Output from -1.0 to 1.0
     */
    public void runRightManual(double percent) {
        climberRight.setControl(manualRequest.withOutput(percent));
        climberLeft.setControl(brakeRequest);
    }

    public void stop() {
        climberLeft.setControl(brakeRequest);
        climberRight.setControl(brakeRequest); 
    }
}