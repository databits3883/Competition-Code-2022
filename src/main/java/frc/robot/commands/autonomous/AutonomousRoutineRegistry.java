package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousRoutineRegistry {
    private static AutonomousRoutineRegistry instance;

    public static AutonomousRoutineRegistry getInstance(){
        if(instance == null){
            instance = new AutonomousRoutineRegistry();
        }
        return instance;
    }

    private ArrayList<Command> routines;
    private SendableChooser<Command> chooser;
    private AutonomousRoutineRegistry(){
        routines = new ArrayList<>();
        chooser = new SendableChooser<Command>();
    }

    public void register(Command routine){
        routines.add(routine);
        chooser.addOption(routine.getName(), routine);
    }


}
