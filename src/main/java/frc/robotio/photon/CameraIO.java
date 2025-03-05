package frc.robotio.photon;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public abstract class CameraIO {
    @AutoLog
    public static class CameraData {
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];
    }

    public final CameraDataAutoLogged data = new CameraDataAutoLogged();

    public abstract void update();
}
