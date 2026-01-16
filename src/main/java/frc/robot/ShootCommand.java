package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;

    public ShootCommand(TurretSubsystem turret, ShooterSubsystem shooter) {
        this.turret = turret;
        this.shooter = shooter;
        addRequirements(turret, shooter);
    }

    @Override
    public void execute() {
        turret.alignToTarget();

        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double distance = calculateDistance(ty);
        double dynamicRPM = shooter.getRPMForDistance(distance);

        shooter.setShooterVelocity(dynamicRPM);

        if (turret.isAligned() && shooter.isAtVelocity() && !shooter.isHopperEmpty()) {
            shooter.runFeeder(1.0);
        } else {
            shooter.runFeeder(0.0);
        }
    }

    private double calculateDistance(double ty) {
        double h1 = 0.5; 
        double h2 = 2.0; 
        double a1 = Math.toRadians(25); 
        double a2 = Math.toRadians(ty);
        return (h2 - h1) / Math.tan(a1 + a2);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHopperEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        turret.stopTurret();
    }
}