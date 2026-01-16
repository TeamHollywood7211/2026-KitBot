package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor = new SparkMax(10, MotorType.kBrushless);
    private final PIDController turretPID = new PIDController(0.04, 0, 0);

    public TurretSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void alignToTarget() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (tv == 1) {
            turretMotor.set(turretPID.calculate(tx, 0));
        } else {
            stopTurret();
        }
    }

    public boolean isAligned() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        return tv == 1 && Math.abs(tx) < 1.0;
    }

    public void stopTurret() {
        turretMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        SmartDashboard.putBoolean("Turret/Aligned", isAligned());
        SmartDashboard.putNumber("Turret/TX", tx);
        SmartDashboard.putBoolean("Turret/Has Target", tv == 1);
    }
}