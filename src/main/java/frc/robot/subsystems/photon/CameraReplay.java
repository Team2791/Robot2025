package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraReplay extends CameraIO {
    public CameraReplay(String name, Transform3d bot2cam) {
        super(name, bot2cam);
    }

    @Override
    protected List<PhotonPipelineResult> results() {
        return List.of();
    }

    @Override
    public void setDriverMode(boolean enabled) {}
}
