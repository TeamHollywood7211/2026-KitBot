package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax feederMotor = new SparkMax(12, MotorType.kBrushless);
    private final DigitalInput hopperSensor = new DigitalInput(0); 
    
    // Corrected for 2026 WPILib Syntax
    private final Debouncer emptyDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    private double targetRPM = 0;

    public void setShooterVelocity(double rpm) {
        targetRPM = rpm;
        shooterMotor.set(rpm / 5676.0); 
    }

    public boolean isAtVelocity() {
        return Math.abs(shooterMotor.getEncoder().getVelocity() - targetRPM) < 100;
    }

    public void runFeeder(double speed) {
        feederMotor.set(speed);
    }

    public boolean isHopperEmpty() {
        return emptyDebouncer.calculate(hopperSensor.get());
    }

    public void stopAll() {
        shooterMotor.stopMotor();
        feederMotor.stopMotor();
    }
}