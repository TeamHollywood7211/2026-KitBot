package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final BooleanSupplier override;
    
    // *UPDATE WITH 2026 COORDINATES FOR THE HUB*
    private final Translation2d BLUE_HUB = new Translation2d(8.27, 4.10);
    private final Translation2d RED_HUB = new Translation2d(8.27, 4.10);

    public ShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, BooleanSupplier override) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.override = override;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        drivetrain.setLimelightPipeline(1); // Switch to Targeting Pipeline
    }

    @Override
    public void execute() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double distance;

        if (tv == 1) {
            // Precision Mode: Use Limelight vertical offset (ty)
            double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            distance = calculateDistance(ty);
        } else {
            // Blind Mode: Calculate distance via Odometry to the Hub
            Translation2d target = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
                == DriverStation.Alliance.Blue ? BLUE_HUB : RED_HUB;
            
            distance = target.getDistance(drivetrain.getStatePose().getTranslation());
        }

        double dynamicRPM = shooter.getRPMForDistance(distance);
        shooter.setLaunchVelocity(dynamicRPM);

        // Only shoot if aligned AND (Vision sees target OR Override is held)
        boolean canShoot = (drivetrain.isAligned() || override.getAsBoolean()) && shooter.isAtVelocity();

        if (canShoot && !shooter.isHopperEmpty()) {
            shooter.runHopper(-1.0);
        } else {
            shooter.runHopper(0.0);
        }
    }

    private double calculateDistance(double ty) {
        double h1 = 0.5; // Limelight Height
        double h2 = 2.64; // Hub Height (Update this based on 2026 Manual)
        double a1 = Math.toRadians(25); // Mount Angle
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
        drivetrain.setLimelightPipeline(0); // Switch back to Localization
    }
}