package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SimulateMatchCommand extends Command {
    
    private final SendableChooser<Command> autoChooser;
    private final Timer realTimeTimer = new Timer();
    
    private enum MatchState { AUTO, TRANSITION, TELEOP, FINISHED }
    private MatchState currentState = MatchState.AUTO;

    public SimulateMatchCommand(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
    }

    @Override
    public void initialize() {
        System.out.println("--- 2026 SIMULATION STARTED ---");
        realTimeTimer.restart();
        currentState = MatchState.AUTO;

        // Force Auto Start
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(20.0); 
        DriverStationSim.notifyNewData(); 

        Command selectedAuto = autoChooser.getSelected();
        if (selectedAuto != null) {
            CommandScheduler.getInstance().schedule(selectedAuto);
        }
    }

    @Override
    public void execute() {
        double elapsed = realTimeTimer.get();

        switch (currentState) {
            case AUTO:
                double autoTimeRemaining = 20.0 - elapsed;
                
                if (autoTimeRemaining <= 0) {
                    System.out.println("--- AUTO ENDED. TRANSITIONING... ---");
                    
                    if (autoChooser.getSelected() != null) {
                        autoChooser.getSelected().cancel();
                    }

                    // ENTER TRANSITION STATE (Disable Robot for 0.5s)
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setEnabled(false); 
                    DriverStationSim.setMatchTime(0.0);
                    
                    currentState = MatchState.TRANSITION;
                    realTimeTimer.restart(); 
                } else {
                    DriverStationSim.setMatchTime(autoTimeRemaining);
                }
                break;

            case TRANSITION:
                // WAIT FOR 0.5 SECONDS (Simulating field delay)
                if (elapsed >= 0.5) {
                    System.out.println("--- TELEOP STARTED ---");
                    
                    // --- FIX IS HERE ---
                    // To set Teleop, we just ensure Auto and Test are FALSE.
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);
                    
                    DriverStationSim.setEnabled(true); 
                    DriverStationSim.setMatchTime(140.0); 
                    
                    currentState = MatchState.TELEOP;
                    realTimeTimer.restart();
                }
                break;

            case TELEOP:
                double teleopTimeRemaining = 140.0 - elapsed; 

                if (teleopTimeRemaining <= 0) {
                    currentState = MatchState.FINISHED;
                } else {
                    DriverStationSim.setMatchTime(teleopTimeRemaining);
                }
                break;

            case FINISHED:
                break;
        }
        DriverStationSim.notifyNewData();
    }

    @Override
    public boolean isFinished() {
        return currentState == MatchState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("--- SIMULATION ENDED ---");
        DriverStationSim.setEnabled(false);
        DriverStationSim.setMatchTime(0);
        DriverStationSim.notifyNewData();
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
}