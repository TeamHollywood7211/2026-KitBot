package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

// Change from ParallelCommandGroup to SequentialCommandGroup
public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        addCommands(
            // 1. Start the flywheel (this finishes instantly)
            shooter.runOnce(() -> shooter.setLaunchVelocity(3500)),
            
            // 2. Wait for spin-up (0.5s)
            new WaitCommand(0.5), 
            
            // 3. Run the shoot command (which likely runs the hopper)
            new ShootCommand(drivetrain, shooter, () -> false)
        );
    }
}
