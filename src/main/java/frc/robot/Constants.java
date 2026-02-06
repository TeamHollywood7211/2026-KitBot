package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Constants {

    public static final class ShooterConstants {
        public static final int kIntakeMotorId = 44;
        public static final int kFlywheelMotorId = 42;
        public static final int kHopperMotorId = 43;
        public static final int kFrontSensorId = 48;
        public static final int kHopperSensorId = 47;

        public static final double kFlywheelkP = 0.11;
        public static final double kFlywheelkS = 0.25;
        public static final double kFlywheelkV = 0.12;
        public static final double kHopperkV = 0.12;
        public static final double kHopperkP = 0.11;
        public static final double kIntakekV = 0.12;
        public static final double kIntakekP = 0.11;

        public static final boolean kFlywheelCurrentLimits = true;
        public static final boolean kHopperCurrentLimits = true;
        public static final boolean kIntakeCurrentLimits = true;

        public static final int kFlywheelStatorCurrentLimit = 60;
        public static final int kHopperStatorCurrentLimit = 40;
        public static final int kIntakeStatorCurrentLimit = 60;

        public static final NeutralModeValue kFlywheelRunMode = NeutralModeValue.Coast;
        public static final NeutralModeValue kHopperRunMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kIntakeRunMode = NeutralModeValue.Brake;

    }

    public static final class ClimberConstants {
        public static final int kClimberRightId = 56;
        public static final int kClimberLeftId = 55;

        // MOTOR ALIGNMENT / INVERSION
        public static final InvertedValue kLeftInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kRightInverted = InvertedValue.Clockwise_Positive;

        // Follower
        // We use (Aligned) here because we are setting the Inversion
        // in the Hardware Config below. 'Positive' will mean 'Up' for both motors.
        public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed;

        // TUNING (Stiffer hold)
        public static final double kClimberkP = 1.0;
        public static final double kClimberkI = 0.0;
        public static final double kClimberkD = 0.0;
        public static final double kClimberkS = 0.25;
        public static final double kClimberkV = 0.12;

        // MOTION MAGIC (Speed Control for Auto)
        // 80 RPS = Full extension in ~1.5 seconds
        public static final double kMotionMagicCruiseVelocity = 80.0;
        public static final double kMotionMagicAcceleration = 160.0; // Reach max speed in 0.5s
        public static final double kMotionMagicJerk = 0.0; // 0 = Disable Jerk smoothing (fastest)

        // POWER
        public static final boolean kClimberCurrentLimits = true;
        public static final int kClimberStatorCurrentLimit = 40; // Needed for lifting weight
        public static final NeutralModeValue kClimberRunMode = NeutralModeValue.Brake;

        // LIMITS
        public static final double kMaxHeightRotations = 122.0;
        // Allow below zero to manually reset climber with robot on
        public static final double kMinHeightRotations = -20.0;
        public static final double kClimbTargetHeight = 20.0;
        public static final boolean kEnableSoftLimits = true;

        // MANUAL SPEED
        // Increased from 1.0 to 50.0 (Much faster response)
        public static final double kManualSpeed = 50.0;
    }
}
