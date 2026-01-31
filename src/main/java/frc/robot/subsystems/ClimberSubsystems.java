// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.CANrange;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import frc.robot.Constants.ClimberConstants;

// public class ClimberSubsystems {
    
// private final TalonFX climberLeft = new TalonFX(ClimberConstants.kClimberLeftId);
// private final TalonFX climberRight = new TalonFX(ClimberConstants.kClimberRightId);

// // climberRight.setControl(new Follower(ClimberConstants.kClimberLeftId, MotorAlignmentValue.ali))
// climberRight.follow(climberLeft);

// }
