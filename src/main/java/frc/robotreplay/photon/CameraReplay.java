package frc.robotreplay.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robotio.photon.CameraIO;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public class CameraReplay extends CameraIO {
    public CameraReplay(String name, Transform3d bot2cam) {
        super(name, bot2cam);
    }

    @Override
    protected List<PhotonPipelineResult> results() {
        return List.of();
    }
}
