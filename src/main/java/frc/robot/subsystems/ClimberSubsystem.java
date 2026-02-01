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
    // 1. Velocity Request (Reused for both motors when needed)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    
    // 2. Follower Request for the Right Motor (Opposed)
    private final Follower followerRequest = new Follower(ClimberConstants.kClimberLeftId, MotorAlignmentValue.Opposed);

    // 3. Stop Request
    private final NeutralOut brakeRequest = new NeutralOut();

    public ClimberSubsystem() {
        // --- 1. Configure LEADER (Left) ---
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        leaderConfigs.Slot0.kP = ClimberConstants.kClimberkP;
        leaderConfigs.Slot0.kS = ClimberConstants.kClimberkS;
        leaderConfigs.Slot0.kV = ClimberConstants.kClimberkV;
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        leaderConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;
        climberLeft.getConfigurator().apply(leaderConfigs);

        // --- 2. Configure FOLLOWER (Right) ---
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        followerConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;
        climberRight.getConfigurator().apply(followerConfigs);
        
        // Start in Follower Mode by default
        climberRight.setControl(followerRequest);
    }

    /**
     * UNIFIED MODE: Left drives, Right follows (Opposed).
     * Call this for normal climbing.
     */
    public void runClimber(double rps) {
        // 1. Set Left to speed
        climberLeft.setControl(velocityRequest.withVelocity(rps));
        
        // 2. FORCE Right to follow Left 
        // (Crucial: This restores the link if we were previously in Manual Mode)
        climberRight.setControl(followerRequest);
    }

    /**
     * MANUAL LEFT ONLY: Moves just the left hook.
     * Right motor will stay in its last state (likely brake).
     */
    public void runLeftManual(double rps) {
        climberLeft.setControl(velocityRequest.withVelocity(rps));
        // We do NOT set right to follow here, so it stays put (or does its own thing)
        climberRight.setControl(brakeRequest); 
    }

    /**
     * MANUAL RIGHT ONLY: Moves just the right hook.
     * This BREAKS the follower link until runClimber() is called again.
     */
    public void runRightManual(double rps) {
        // We send a direct velocity command to Right, overriding the Follower
        climberRight.setControl(velocityRequest.withVelocity(rps));
        climberLeft.setControl(brakeRequest);
    }

    public void stop() {
        climberLeft.setControl(brakeRequest);
        climberRight.setControl(brakeRequest); 
    }

    @Override
    public void periodic() {
        // Monitor status if needed
    }
}