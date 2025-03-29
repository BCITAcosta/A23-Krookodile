package frc.robot.subsystems;

import java.util.Hashtable;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Enumeration;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class Autonomous extends SubsystemBase{
    private static Autonomous m_autonomous;

    private SendableChooser<Command> autoRoutineChooser;
    private Hashtable<String, Command> autoRoutines;

    private PathPlannerAuto driveOut;


    public Autonomous(){
        defineAutoPaths();
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineChooser = new SendableChooser<Command>();
        setupAutoRoutines();
        setupAutoSelector();
    }

    public static Autonomous getInstance(){
        if(m_autonomous == null){
            m_autonomous = new Autonomous();
        }
        return m_autonomous;
    }

    public void setupAutoRoutines(){
        autoRoutines.put("Drive Out", driveOut);
    }

    public void setupAutoSelector(){
        Enumeration<String> e = autoRoutines.keys();

        while(e.hasMoreElements()){
            String autoRoutineName = e.nextElement();
            autoRoutineChooser.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        };

        SmartDashboard.putData("Auto Routines", autoRoutineChooser);
    }

    public Command returnAutonomousCommand(){
        return autoRoutineChooser.getSelected();
    }

    public void defineAutoPaths(){
        driveOut = new PathPlannerAuto("DriveOutOnly");
    }

}