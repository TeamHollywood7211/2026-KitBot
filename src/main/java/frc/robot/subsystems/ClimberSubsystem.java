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
    // 1. Velocity Request for the Leader
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    
    // 2. Follower Request for the Right Motor
    private final Follower followerRequest = new Follower(ClimberConstants.kClimberLeftId, MotorAlignmentValue.Opposed);

    // 3. Stop Request
    private final NeutralOut brakeRequest = new NeutralOut();

    public ClimberSubsystem() {
        // --- 1. Configure LEADER (Left) ---
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();

        // PID Gains (from your Constants)
        leaderConfigs.Slot0.kP = ClimberConstants.kClimberkP;
        leaderConfigs.Slot0.kS = ClimberConstants.kClimberkS;
        leaderConfigs.Slot0.kV = ClimberConstants.kClimberkV;

        // Current Limits
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;

        // Neutral Mode (Brake)
        leaderConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;

        // Apply configs to Left
        climberLeft.getConfigurator().apply(leaderConfigs);

        // Configure FOLLOWER (Right) ---
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        
        // Follower needs same limits and brake mode for safety
        followerConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        followerConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;

        // Apply configs to Right
        climberRight.getConfigurator().apply(followerConfigs);

        // Activate Follower Mode ---
        // This tells the Right motor: "Follow Left, but Opposed"
        climberRight.setControl(followerRequest);
    }

    /**
     * Runs the climber at a specific velocity (Rotations Per Second).
     * The Right motor will automatically follow.
     * @param rps Rotations Per Second
     */
    public void runClimber(double rps) {
        climberLeft.setControl(velocityRequest.withVelocity(rps));
    }

    public void stop() {
        climberLeft.setControl(brakeRequest);
        climberRight.setControl(brakeRequest); 
    }

    @Override
    public void periodic() {
        // Optional: Monitor temps or status here
    }
}