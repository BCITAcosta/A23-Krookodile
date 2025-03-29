package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    private static Elevator elevator;

    public Elevator(){

    }

    public static Elevator getInstance(){
        if (elevator == null){
            elevator = new Elevator();
        }
        
        return elevator;
    }
}
