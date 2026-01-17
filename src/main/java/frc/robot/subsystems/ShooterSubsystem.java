package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX intakeFlywheelMotor = new TalonFX(42, "mechanisms");
    private final TalonFX hopperMotor = new TalonFX(43, "mechanisms");
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final CANrange hopperSensor = new CANrange(47);
    // private final CANrange frontSensor = new CANrange(48);
    private final Debouncer emptyDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private double targetRPM = 0;

    public ShooterSubsystem() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        TalonFXConfiguration hopperConfigs = new TalonFXConfiguration();

        intakeConfigs.Slot0.kV = 0.12; 
        intakeConfigs.Slot0.kP = 0.11; 
        intakeConfigs.Slot0.kS = 0.25; 

        hopperConfigs.Slot0.kV = 0.12; 
        hopperConfigs.Slot0.kP = 0.15; 

        intakeFlywheelMotor.getConfigurator().apply(intakeConfigs);
        hopperMotor.getConfigurator().apply(hopperConfigs);
        
        CANrangeConfiguration rangeConfigs = new CANrangeConfiguration();
        // Set threshold: detects if object is within 0.15 meters (15cm)
        rangeConfigs.ProximityParams.ProximityThreshold = 0.15; 
        hopperSensor.getConfigurator().apply(rangeConfigs);
       
        // Map setup
        rpmMap.put(1.0, 2500.0);
        rpmMap.put(2.0, 3100.0);
        rpmMap.put(3.0, 3500.0);
        rpmMap.put(4.0, 4200.0);
        rpmMap.put(5.0, 5000.0);
    }

    // --- Added Methods to Resolve Compiler Errors ---

    public double getRPMForDistance(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public boolean isHopperEmpty() {
        // refresh() updates the signal from the CAN bus
        boolean isBallPresent = hopperSensor.getIsDetected().refresh().getValue();
        return emptyDebouncer.calculate(!isBallPresent);
    }

    public void stopAll() {
        intakeFlywheelMotor.stopMotor();
        hopperMotor.stopMotor();
        targetRPM = 0;
    }

    public void setIntakeMode(double percent) {
        // Use duty cycle for basic percent-output intake
        intakeFlywheelMotor.set(percent);
        hopperMotor.set(percent);
    }

    public void setOuttakeMode(double percent) {
        intakeFlywheelMotor.set(-percent);
        hopperMotor.set(-percent);
    }

    // --- Velocity Control Methods ---

    public void setLaunchVelocity(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0; 
        intakeFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void runHopper(double speed) {
        // If speed is a small percent (0.0 to 1.0), use .set()
        // If speed is RPM, use setControl with rps conversion
        hopperMotor.set(speed);
    }

    public boolean isAtVelocity() {
        double actualRPM = intakeFlywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        return Math.abs(actualRPM - targetRPM) < 100;
    }

    @Override
    public void periodic() {
        double currentRPM = intakeFlywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", currentRPM);
        SmartDashboard.putBoolean("Shooter/At Velocity", isAtVelocity());
        SmartDashboard.putBoolean("Shooter/Hopper Empty", isHopperEmpty());
    }
}
