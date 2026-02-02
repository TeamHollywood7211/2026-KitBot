package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX climberLeft = new TalonFX(ClimberConstants.kClimberLeftId);
    private final TalonFX climberRight = new TalonFX(ClimberConstants.kClimberRightId);

    // --- Control Requests ---
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final Follower followerRequest = new Follower(ClimberConstants.kClimberLeftId, MotorAlignmentValue.Opposed);
    private final NeutralOut brakeRequest = new NeutralOut();

    // --- Constructor ---
    public ClimberSubsystem() {
        // 1. Configure LEADER (Left)
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        leaderConfigs.Slot0.kP = ClimberConstants.kClimberkP;
        leaderConfigs.Slot0.kS = ClimberConstants.kClimberkS;
        leaderConfigs.Slot0.kV = ClimberConstants.kClimberkV;
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        leaderConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;

        // Soft Limits
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        climberLeft.getConfigurator().apply(leaderConfigs);

        // 2. Configure FOLLOWER (Right)
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.kClimberStatorCurrentLimit;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = ClimberConstants.kClimberCurrentLimits;
        followerConfigs.MotorOutput.NeutralMode = ClimberConstants.kClimberRunMode;
        
        // Redundant Soft Limits
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberConstants.kEnableSoftLimits;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMinHeightRotations;
        followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberConstants.kEnableSoftLimits;

        climberRight.getConfigurator().apply(followerConfigs);
        
        zeroSensors();
        climberRight.setControl(followerRequest);
    }

    public void zeroSensors() {
        climberLeft.setPosition(0);
        climberRight.setPosition(0);
    }

    // --- AUTO COMMAND HELPER METHODS ---

    /**
     * EXTEND COMMAND: Moves climbers to the absolute maximum height.
     * Call this at the start of the climb sequence (e.g., in Auto).
     */
    public void extendMax() {
        moveToPosition(ClimberConstants.kMaxHeightRotations);
    }

    /**
     * RETRACT COMMAND: Pulls climbers down to the "Climb Target" height.
     * Call this to actually lift the robot.
     */
    public void retractToClimb() {
        moveToPosition(ClimberConstants.kClimbTargetHeight);
    }

    // --- Internal Logic ---

    public void moveToPosition(double targetRotations) {
        double safeTarget = Math.max(targetRotations, ClimberConstants.kMinHeightRotations);
        safeTarget = Math.min(safeTarget, ClimberConstants.kMaxHeightRotations);
        
        climberLeft.setControl(positionRequest.withPosition(safeTarget));
        climberRight.setControl(followerRequest);
    }

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
    
    public double getLeftHeight() { return climberLeft.getPosition().getValueAsDouble(); }
    public double getRightHeight() { return climberRight.getPosition().getValueAsDouble(); }

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Climber Height", getLeftHeight());
    }
}