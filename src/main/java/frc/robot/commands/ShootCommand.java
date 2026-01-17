package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final BooleanSupplier override;

    public ShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, BooleanSupplier override) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.override = override;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double distance = calculateDistance(ty);
        double dynamicRPM = shooter.getRPMForDistance(distance);

        shooter.setLaunchVelocity(dynamicRPM);

        boolean canShoot = (drivetrain.isAligned() || override.getAsBoolean()) && shooter.isAtVelocity();

        if (canShoot && !shooter.isHopperEmpty()) {
            shooter.runHopper(-1.0);
        } else {
            shooter.runHopper(0.0);
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
    }
}