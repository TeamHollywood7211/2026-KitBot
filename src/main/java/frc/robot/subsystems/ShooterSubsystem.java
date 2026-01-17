package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax intakeFlywheelMotor = new SparkMax(42, MotorType.kBrushless);
    private final SparkMax hopperMotor = new SparkMax(43, MotorType.kBrushless);
    private final DigitalInput hopperSensor = new DigitalInput(0); 
    private final Debouncer emptyDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private double targetRPM = 0;

    @SuppressWarnings("removal")
    public ShooterSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(60);
        
        intakeFlywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rpmMap.put(1.0, 2500.0);
        rpmMap.put(2.0, 3100.0);
        rpmMap.put(3.0, 3500.0);
        rpmMap.put(4.0, 4200.0);
        rpmMap.put(5.0, 5000.0);
    }

    public double getRPMForDistance(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public void setLaunchVelocity(double rpm) {
        targetRPM = rpm;
        intakeFlywheelMotor.set(rpm / 5676.0); 
    }

    public void runHopper(double speed) {
        hopperMotor.set(speed);
    }

    public void setIntakeMode(double speed) {
        intakeFlywheelMotor.set(speed);
        hopperMotor.set(speed);
    }

    public void setOuttakeMode(double speed) {
        intakeFlywheelMotor.set(-speed);
        hopperMotor.set(-speed);
    }

    public boolean isAtVelocity() {
        return Math.abs(intakeFlywheelMotor.getEncoder().getVelocity() - targetRPM) < 100;
    }

    public boolean isHopperEmpty() {
        return emptyDebouncer.calculate(hopperSensor.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", intakeFlywheelMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/At Velocity", isAtVelocity());
        SmartDashboard.putBoolean("Shooter/Hopper Empty", isHopperEmpty());
    }

    public void stopAll() {
        intakeFlywheelMotor.stopMotor();
        hopperMotor.stopMotor();
    }
}