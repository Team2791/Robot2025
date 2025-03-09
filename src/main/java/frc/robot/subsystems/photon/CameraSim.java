package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import frc.robot.util.WorldSimulator;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public class CameraSim extends CameraIO {
	final PhotonCamera camera;
	final PhotonCameraSim cameraSim;

	public CameraSim(String name, Transform3d bot2cam) {
		super(name, bot2cam);

		camera = new PhotonCamera(name);
		cameraSim = new PhotonCameraSim(camera, VisionConstants.kSimCameraProps);

		WorldSimulator.getInstance().addCamera(cameraSim, bot2cam);

		cameraSim.enableDrawWireframe(true);
	}

	@Override
	protected List<PhotonPipelineResult> results() {
		return camera.getAllUnreadResults();
	}
}
