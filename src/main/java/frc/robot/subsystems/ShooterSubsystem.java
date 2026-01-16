package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax feederMotor = new SparkMax(12, MotorType.kBrushless);
    private final DigitalInput hopperSensor = new DigitalInput(0); 
    private final Debouncer emptyDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
    
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private double targetRPM = 0;

    public ShooterSubsystem() {
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.smartCurrentLimit(60);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rpmMap.put(1.0, 2500.0);
        rpmMap.put(2.0, 3100.0);
        rpmMap.put(3.0, 3500.0);
        rpmMap.put(4.0, 4200.0);
        rpmMap.put(5.0, 5000.0);
    }

    public double getRPMForDistance(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/At Velocity", isAtVelocity());
        SmartDashboard.putBoolean("Shooter/Hopper Empty", isHopperEmpty());
    }

    public void stopAll() {
        shooterMotor.stopMotor();
        feederMotor.stopMotor();
    }
}