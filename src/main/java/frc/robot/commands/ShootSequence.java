package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GameSwerveDrivetrain; // Update Import
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
    // Constructor must accept the specific GameSwerveDrivetrain
    public ShootSequence(GameSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        addCommands(
            // 1. Start the flywheel (this finishes instantly)
            shooter.runOnce(() -> shooter.setLaunchVelocity(3600)),
            
            // 2. Wait for spin-up (0.5s)
            new WaitCommand(0.5), 
            
            // 3. Run the shoot command
            // Now passing the correct GameSwerveDrivetrain type
            new ShootCommand(drivetrain, shooter, () -> false)
        );
    }
}