package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

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

        public static final int kClimberRightId = 55;
        public static final int kClimberLeftId = 56;

        public static final double kClimberkP = 0.11;
        public static final double kClimberkS = 0.25;
        public static final double kClimberkV = 0.12;

        public static final boolean kClimberCurrentLimits = true;

        public static final int kClimberStatorCurrentLimit = 20;

        public static final NeutralModeValue kClimberRunMode = NeutralModeValue.Brake;

        public static final double kMaxHeightRotations = 80.0;
        public static final double kMinHeightRotations = 0.0;
        public static final double kClimbTargetHeight = 5.0;

        public static final boolean kEnableSoftLimits = true;

        public static final double kManualSpeed = 10.0;

    }
}
