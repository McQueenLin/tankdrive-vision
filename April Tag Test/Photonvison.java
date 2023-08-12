package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Photonvison {
    static PhotonCamera camera = new PhotonCamera("team4");

    public static double getDistance(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d ddd = target.getBestCameraToTarget();
            
            return ddd.getX();
        } else{
            return 0;
        }
        
    }

    public static double getRotation(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d ddd = target.getBestCameraToTarget();
    
            return Math.abs(ddd.getRotation().getZ()*100-120);
        } else{
            return 0;
        }
    }

}
