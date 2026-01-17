package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        addCommands(
            shooter.runOnce(() -> shooter.setLaunchVelocity(3500)),
            new SequentialCommandGroup(
                new WaitCommand(0.5), 
                new ShootCommand(drivetrain, shooter, () -> false)
            )
        );
    }
}