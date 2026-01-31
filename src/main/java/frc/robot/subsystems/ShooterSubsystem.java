package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Add Utils for Simulation Check
import com.ctre.phoenix6.Utils; 

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

public class ShooterSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX intakeFlywheelMotor = new TalonFX(42);
    private final TalonFX hopperMotor = new TalonFX(43);
    
    // Sensors
    private final CANrange hopperSensor = new CANrange(47); 
    private final CANrange frontSensor = new CANrange(48);  
    
    // Logic
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final Debouncer jamDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    
    private double targetRPM = 0;
    private double manualRPM = 3500; 

    private double manualSpeed = 0.68;

    
    // --- NOTIFICATION STATE ---
    private boolean wasJammed = false;

    public ShooterSubsystem() {
        // --- MOTOR CONFIG ---
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        TalonFXConfiguration hopperConfigs = new TalonFXConfiguration();

        intakeConfigs.Slot0.kV = 0.12; 
        intakeConfigs.Slot0.kP = 0.11; 
        intakeConfigs.Slot0.kS = 0.25; 
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        hopperConfigs.Slot0.kV = 0.12; 
        hopperConfigs.Slot0.kP = 0.11;
        hopperConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        intakeFlywheelMotor.getConfigurator().apply(intakeConfigs);
        hopperMotor.getConfigurator().apply(hopperConfigs);
        
        // --- SENSOR CONFIG ---
        CANrangeConfiguration rangeConfigs = new CANrangeConfiguration();
        rangeConfigs.ProximityParams.ProximityThreshold = 0.15; 
        
        hopperSensor.getConfigurator().apply(rangeConfigs);
        frontSensor.getConfigurator().apply(rangeConfigs);
       
        // --- DISTANCE MAP ---
        rpmMap.put(1.0, 3600.0);
        rpmMap.put(2.0, 3700.0);
        rpmMap.put(3.0, 3800.0);
        rpmMap.put(4.0, 3900.0);
        rpmMap.put(5.0, 4000.0);
    }

    public double getRPMForDistance(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    // --- SENSOR METHODS ---
    public boolean isJammed() {
        boolean rawDetection = hopperSensor.getIsDetected().refresh().getValue();
        return jamDebouncer.calculate(rawDetection);
    }

    public boolean isHumanLoadDetected() {
        return frontSensor.getIsDetected().refresh().getValue();
    }

    public boolean isAtVelocity() {
        double actualRPM = intakeFlywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        // In Sim, we might be perfect, so allow a small error
        return Math.abs(actualRPM - targetRPM) < 100;
    }

    // --- MANUAL CONTROLS ---
    public void incrementManualRPM() { manualRPM += 50; }
    public void decrementManualRPM() { manualRPM -= 50; }
    public double getManualRPM() { return manualRPM; }

    // --- MOTORS ---
    public void stopAll() {
        intakeFlywheelMotor.stopMotor();
        hopperMotor.stopMotor();
        targetRPM = 0;
    }

    public void setIntakeMode(double percent) {
        intakeFlywheelMotor.set(percent);
        hopperMotor.set(percent);
    }

    public void setOuttakeMode(double percent) {
        intakeFlywheelMotor.set(-percent);
        hopperMotor.set(-percent);
    }

    public void setEjectMode(double percent) {
        intakeFlywheelMotor.set(-percent);
        hopperMotor.set(percent);
    }

    public void setLaunchVelocity(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0; 
        intakeFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void runHopper(double speed) {
        hopperMotor.set(speed);
    }

    public void runIntake(double speed) {
        intakeFlywheelMotor.set(speed);
    }

    public void setLow(){
        manualSpeed=0.4;
    }

    public void setMedium(){
        manualSpeed=0.55;
    }

    public void setHigh(){
        manualSpeed=0.68;
    }

    public void setSuperHigh(){
        manualSpeed=1.0;
    }



    @Override
    public void periodic() {
        // --- SIMULATION PHYSICS HACK ---
        if (Utils.isSimulation()) {
            double simulatedRPS = targetRPM / 60.0;
            intakeFlywheelMotor.getSimState().setRotorVelocity(simulatedRPS);
        }
        
        // --- JAM NOTIFICATION LOGIC ---
        boolean currentlyJammed = isJammed();
        
        if (currentlyJammed && !wasJammed) {
            // RISING EDGE: Just became jammed
            Elastic.Notification alert = new Elastic.Notification(
                NotificationLevel.ERROR, 
                "SHOOTER JAMMED!", 
                "Hopper blocked for >0.5s. Reverse Intake!"
            );
            Elastic.sendNotification(alert);
            
        } else if (!currentlyJammed && wasJammed) {
            // FALLING EDGE: Jam cleared
            Elastic.Notification clear = new Elastic.Notification(
                NotificationLevel.INFO, 
                "Jam Cleared", 
                "Shooter is ready."
            );
            Elastic.sendNotification(clear);
        }
        
        wasJammed = currentlyJammed; // Update state for next loop
        // ------------------------------

        double currentRPM = intakeFlywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", currentRPM);
        SmartDashboard.putNumber("Shooter/Manual Setpoint", manualRPM);
        SmartDashboard.putBoolean("Shooter/At Velocity", isAtVelocity());
        SmartDashboard.putBoolean("Shooter/Is Jammed", currentlyJammed);
        SmartDashboard.putBoolean("Shooter/Human Load Detect", isHumanLoadDetected());
    }
}