package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    PhotonCamera camera1, camera2;
    

    public Vision(){
        camera1 = new PhotonCamera("camera1");
        camera2 = new PhotonCamera("camera2");
    }

    @Override
    public void periodic(){

    }
    
}
