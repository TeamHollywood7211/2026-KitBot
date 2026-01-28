package frc.robot;

/*
 * =========================================================================================
 * 🚨  WARNING: DO NOT OVERWRITE THIS FILE WITH PHOENIX TUNER GENERATED CODE  🚨
 * =========================================================================================
 * * This file contains custom logic for:
 * - Two Controller Setup
 * - GameSwerveDrivetrain
 * - Shooter Subsystem Integration
 * - PathPlanner Registration
 * - Elastic Dashboard Notifications
 * - Game Phase Monitoring
 * - SIMULATION MATCH CONTROL (This was the missing piece!)
 * * =========================================================================================
 */

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// --- REMOTE LAYOUT IMPORTS ---
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
// --- SIM COMMAND IMPORT ---
import frc.robot.commands.SimulateMatchCommand; 

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GameSwerveDrivetrain; 
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.GamePhaseSubsystem;
import frc.robot.commands.ShootCommand;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

public class RobotContainer {
    // Tuning Variables
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private double TrainingWheels = 1; 

    // --- CONTROLLERS ---
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

    // Drive Request Type
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // --- SUBSYSTEMS ---
    public final GameSwerveDrivetrain drivetrain = new GameSwerveDrivetrain(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight
    );

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    
    // Game Phase Manager
    @SuppressWarnings("unused")
    private final GamePhaseSubsystem gamePhase = new GamePhaseSubsystem();
    
    // Auto Chooser
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // 1. Start USB Camera
        //CameraServer.startAutomaticCapture();

        // 2. Start Remote Layout Server (Port 5800)
        try {
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        } catch (Exception e) {
            System.out.println("Warning: Could not start Elastic Layout Server: " + e.getMessage());
        }

        // 3. Send Notification
        Elastic.sendNotification(new Elastic.Notification(
            NotificationLevel.INFO, 
            "System Online", 
            "Dual-Controller Mode Active"
        ));

        // --- PATHPLANNER ---
        NamedCommands.registerCommand("StartIntake", shooter.runOnce(() -> shooter.setIntakeMode(0.6)));
        NamedCommands.registerCommand("StopShooter", shooter.runOnce(() -> shooter.stopAll()));
        
        NamedCommands.registerCommand("Shoot", 
            shooter.run(() -> {
                shooter.setLaunchVelocity(3600); 
                shooter.runHopper(-1.0);         
            }));
            
        

        // --- INIT SUBSYSTEMS ---
        drivetrain.configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // --- START SIM BUTTON (THE MISSING PIECE) ---
        // This puts the command onto the dashboard so you can drag it as a widget!
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            SmartDashboard.putData("START SIM MATCH", new SimulateMatchCommand(autoChooser));
        }

        configureBindings();
    }

    private void configureBindings() {
        // DRIVER (Joystick 0)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // 1. Get Alliance Color (Default to Blue if unknown)
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);

                // 2. Determine Perspective Multiplier
                // Blue Alliance: Forward is +X (Away). Invert = 1.0
                // Red Alliance:  Forward is -X (Away). Invert = -1.0
                double invert = (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) ? -1.0 : 1.0;

                return drive
                    // VELOCITY X (Forward/Back)
                    // We negate getLeftY() because 'W' (or Stick Up) gives -1.0.
                    // -(-1) = +1 (Forward).
                    // We multiply by 'invert' to handle Red vs Blue perspective.
                    .withVelocityX(-xLimiter.calculate(driverJoystick.getLeftY()) * MaxSpeed * TrainingWheels * invert)
                    
                    // VELOCITY Y (Left/Right)
                    // We negate getLeftX() because 'A' (or Stick Left) gives -1.0.
                    // -(-1) = +1 (Field Left).
                    // We multiply by 'invert' so "Left" is always "Driver's Left".
                    .withVelocityY(-yLimiter.calculate(driverJoystick.getLeftX()) * MaxSpeed * TrainingWheels * invert) 
                    
                    // ROTATION (Spin)
                    // Rotation is absolute (CCW is positive).
                    // 'D' or Right Stick usually gives +1, so we negate it to make Left Stick (+Spin).
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate);
            })
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );
        
        driverJoystick.leftBumper().and(driverJoystick.back().negate()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverJoystick.back().and(driverJoystick.leftBumper()).onTrue(drivetrain.runOnce(drivetrain::resetPoseToLimelight));

        // operatorJoystick.rightTrigger().whileTrue(
        //     new ShootCommand(drivetrain, shooter, () -> operatorJoystick.rightBumper().getAsBoolean())
        // );

        // operatorJoystick.a().and(operatorJoystick.rightBumper()).whileTrue(
        //     shooter.runOnce(() -> shooter.runHopper(-1.0)) 
        // ).onFalse(shooter.runOnce(() -> shooter.runHopper(0.0)));


        //SHOOT
        //operatorJoystick.leftTrigger().whileTrue(new ShootCommand(drivetrain, shooter,() -> true));
        operatorJoystick.rightTrigger().whileTrue( shooter.runOnce( () -> shooter.runIntake(0.68) ) ).onFalse( shooter.runOnce(() ->shooter.runIntake(0.0)));

        //SPOOL
        operatorJoystick.leftTrigger().whileTrue( shooter.runOnce( () -> shooter.runHopper(-0.4) ) ).onFalse( shooter.runOnce(() ->shooter.runHopper(0.0)));


        operatorJoystick.x().whileTrue( shooter.runOnce( () -> shooter.setLow()));
        operatorJoystick.a().whileTrue( shooter.runOnce( () -> shooter.setMedium()));
        operatorJoystick.b().whileTrue( shooter.runOnce( () -> shooter.setHigh()));
        operatorJoystick.y().whileTrue( shooter.runOnce( () -> shooter.setSuperHigh()));


        operatorJoystick.povUp().onTrue(shooter.runOnce(shooter::incrementManualRPM));
        operatorJoystick.povDown().onTrue(shooter.runOnce(shooter::decrementManualRPM));

        driverJoystick.rightTrigger().whileTrue(shooter.run(
            () -> shooter.setIntakeMode(0.8)).finallyDo(shooter::stopAll)
        );

        operatorJoystick.b().whileTrue(shooter.run(
            () -> shooter.setEjectMode(1)).finallyDo(shooter::stopAll)
        );

        driverJoystick.b().whileTrue(shooter.run(
            () -> shooter.setEjectMode(1)).finallyDo(shooter::stopAll)
        );

        operatorJoystick.x().whileTrue(shooter.run(
            () -> shooter.setOuttakeMode(0.4)).finallyDo(shooter::stopAll)
        );

        operatorJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.setLimelightLED(true)))
                                .onFalse(drivetrain.runOnce(() -> drivetrain.setLimelightLED(false)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}