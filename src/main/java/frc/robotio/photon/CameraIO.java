package frc.robotio.photon;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.targeting.PhotonPipelineResult;

public abstract class CameraIO {
    public static class CameraData {
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];
    }

    public CameraIO(String name, Transform3d bot2cam) {
        this.name = name;
        this.bot2cam = bot2cam;
    }

    public final String name;
    public final Transform3d bot2cam;
    public final CameraData data = new CameraData();

    public abstract void update();
}
