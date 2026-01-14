package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);
    private final PIDController pid = new PIDController(0.04, 0, 0.002);

    public TurretSubsystem() {
        pid.setTolerance(1.0); // Degree tolerance
    }

    public void alignToTarget() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (tv == 1) {
            motor.set(pid.calculate(tx, 0));
        } else {
            motor.stopMotor();
        }
    }

    public boolean isAligned() {
        return pid.atSetpoint();
    }

    public void stopTurret() {
        motor.stopMotor();
    }
}