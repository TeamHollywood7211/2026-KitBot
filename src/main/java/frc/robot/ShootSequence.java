package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(TurretSubsystem turret, ShooterSubsystem shooter) {
        // addCommands(
        //     // 1. Always keep the shooter spinning at target RPM
        //     shooter.runOnce(() -> shooter.setShooterVelocity(3500)),

        //     // 2. Run the alignment and feeding sequence in parallel
        //     new SequentialCommandGroup(
        //         // Wait for the drivetrain to settle at the end of the path
        //         new WaitCommand(0.5), 
        //         // Run the actual shooting logic until hopper is empty
        //         new ShootCommand(turret, shooter)
        //     )
        // );
    }
}