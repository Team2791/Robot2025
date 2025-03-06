package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robotio.photon.CameraIO;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public class Camera extends CameraIO {
    final PhotonCamera camera;

    public Camera(String name, Transform3d bot2cam) {
        super(name, bot2cam);
        camera = new PhotonCamera(name);
    }

    @Override
    protected List<PhotonPipelineResult> results() {
        return camera.getAllUnreadResults();
    }
}
