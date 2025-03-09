package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import frc.robotio.CameraDataAutoLogged;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;

public abstract class CameraIO {
	public record VisionMeasurement(Pose3d estimate, double timestamp) {
		public Pose2d estimate2() {
			return estimate.toPose2d();
		}
	}

	@AutoLog
	public static class CameraData {
		public VisionMeasurement[] measurements = new VisionMeasurement[0];
	}

	public CameraIO(String name, Transform3d bot2cam) {
		this.name = name;
		this.estimator = new PhotonPoseEstimator(
			VisionConstants.AprilTag.kLayout,
			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			bot2cam
		);
	}

	private final ArrayList<VisionMeasurement> measurements = new ArrayList<>();
	private final PhotonPoseEstimator estimator;
	private PhotonPipelineResult latestResult;

	public final CameraDataAutoLogged data = new CameraDataAutoLogged();
	public final String name;

	protected abstract List<PhotonPipelineResult> results();

	public void update() {
		List<PhotonPipelineResult> results = results();

		for (PhotonPipelineResult result : results) {
			estimator.update(result).ifPresent(this::addVisionMeasurement);
		}

		data.measurements = measurements.toArray(VisionMeasurement[]::new);
		latestResult = results.isEmpty() ? null : results.get(0);

		measurements.clear();
	}

	void addVisionMeasurement(EstimatedRobotPose estimate) {
		measurements.add(new VisionMeasurement(estimate.estimatedPose, estimate.timestampSeconds));
	}

	/**
	 * Get the latest result from the camera. May be null
	 *
	 * @return the latest result from the camera
	 */
	public PhotonPipelineResult getLatestResult() { return latestResult; }
}
