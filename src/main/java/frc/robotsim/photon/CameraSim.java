package frc.robotsim.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robotio.photon.CameraIO;
import frc.robotsim.globals.WorldSimulator;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSim extends CameraIO {
    final PhotonCamera camera;
    final PhotonCameraSim cameraSim;

    public CameraSim(String name, Transform3d bot2cam) {
        super(name, bot2cam);

        camera = new PhotonCamera(name);
        cameraSim = new PhotonCameraSim(camera);

        WorldSimulator.getInstance().addCamera(cameraSim, bot2cam);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void update() {
        this.data.results = cameraSim.getCamera().getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
    }
}
