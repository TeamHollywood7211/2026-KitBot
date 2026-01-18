package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GameSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final GameSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final BooleanSupplier manualOverride;
    
    private final Translation2d BLUE_HUB = new Translation2d(8.27, 4.10);
    private final Translation2d RED_HUB = new Translation2d(8.27, 4.10);

    public ShootCommand(GameSwerveDrivetrain drivetrain, ShooterSubsystem shooter, BooleanSupplier manualOverride) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.manualOverride = manualOverride;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (!manualOverride.getAsBoolean()) {
            drivetrain.setLimelightPipeline(1);
        }
    }

    @Override
    public void execute() {
        if (manualOverride.getAsBoolean()) {
            shooter.setLaunchVelocity(shooter.getManualRPM());
            shooter.runHopper(0.0);
            return; 
        }

        double tv = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0);
        double distance;

        if (tv == 1) {
            double ty = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ty").getDouble(0);
            distance = calculateDistance(ty);
        } else {
            Translation2d target = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
                == DriverStation.Alliance.Blue ? BLUE_HUB : RED_HUB;
            distance = target.getDistance(drivetrain.getStatePose().getTranslation());
        }

        shooter.setLaunchVelocity(shooter.getRPMForDistance(distance));

        // Auto-Feed Condition: Aligned + At Speed.
        // REMOVED "&& !isHopperEmpty()" because the sensor can't tell us that reliably anymore.
        // We rely on the driver to release the trigger.
        if (drivetrain.isAligned() && shooter.isAtVelocity()) {
            shooter.runHopper(-1.0);
        } else {
            shooter.runHopper(0.0);
        }
    }

    private double calculateDistance(double ty) {
        double h1 = 0.5; 
        double h2 = 2.64; 
        double a1 = Math.toRadians(25); 
        double a2 = Math.toRadians(ty);
        return (h2 - h1) / Math.tan(a1 + a2);
    }

    @Override
    public boolean isFinished() {
        // Run forever until button release (Command Interrupted)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
        drivetrain.setLimelightPipeline(0);
    }
}