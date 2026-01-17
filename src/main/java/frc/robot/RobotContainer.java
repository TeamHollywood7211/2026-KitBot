package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootSequence;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private double TrainingWheels = 0.4;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", new ShootSequence(drivetrain, shooter));
        
        NamedCommands.registerCommand("StartIntake", shooter.runOnce(() -> shooter.setIntakeMode(0.6)));
        
        NamedCommands.registerCommand("StopShooter", shooter.runOnce(() -> shooter.stopAll()));

        NamedCommands.registerCommand("Intake3Seconds", shooter.run(() -> shooter.setIntakeMode(0.6)).withTimeout(3.0));

        NamedCommands.registerCommand("Outtake", shooter.run(
            () -> shooter.setOuttakeMode(0.4)).finallyDo(shooter::stopAll)
        );

        drivetrain.configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY()) * MaxSpeed * TrainingWheels) 
                    .withVelocityY(yLimiter.calculate(-joystick.getLeftX()) * MaxSpeed * TrainingWheels) 
                    .withRotationalRate(
                        joystick.rightTrigger().getAsBoolean() 
                        ? drivetrain.getAutoHeadingRate() 
                        : -joystick.getRightX() * MaxAngularRate
                    )
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        joystick.leftTrigger().whileTrue(shooter.run(
            () -> shooter.setIntakeMode(0.6)).finallyDo(shooter::stopAll)
        );

        joystick.x().whileTrue(shooter.run(
            () -> shooter.setOuttakeMode(0.4)).finallyDo(shooter::stopAll)
        );

        joystick.rightTrigger().whileTrue(
            drivetrain.runOnce(() -> drivetrain.setLimelightLED(true))
            .andThen(new ShootCommand(drivetrain, shooter, () -> joystick.rightBumper().getAsBoolean()))
            .finallyDo(() -> drivetrain.setLimelightLED(false))
        );

        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.setLimelightLED(true)))
                        .onFalse(drivetrain.runOnce(() -> drivetrain.setLimelightLED(false)));
        
        joystick.back().and(joystick.leftBumper()).onTrue(drivetrain.runOnce(drivetrain::resetPoseToLimelight));
        joystick.leftBumper().and(joystick.back().negate()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}