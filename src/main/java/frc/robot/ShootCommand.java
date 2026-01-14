package frc.robot.commands;

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
        shooter.setShooterVelocity(3500);

        if (turret.isAligned() && shooter.isAtVelocity() && !shooter.isHopperEmpty()) {
            shooter.runFeeder(1.0);
        } else {
            shooter.runFeeder(0.0);
        }
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