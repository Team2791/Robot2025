package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public abstract class CameraIO {
    public record VisionMeasurement(Pose3d estimate, Matrix<N3, N1> stdDevs, double timestamp) {
        public Pose2d estimate2() {
            return estimate.toPose2d();
        }
    }

    @AutoLog
    public static class CameraData {
        public VisionMeasurement measurement = null;
    }

    public CameraIO(String name, Transform3d bot2cam) {
        this.name = name;
        this.estimator = new PhotonPoseEstimator(
            VisionConstants.AprilTag.kLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            bot2cam
        );
    }

    private final PhotonPoseEstimator estimator;
    private PhotonPipelineResult latestResult;
    private Matrix<N3, N1> stdDevs;

    public final CameraDataAutoLogged data = new CameraDataAutoLogged();
    public final String name;

    protected abstract List<PhotonPipelineResult> results();

    public void update() {
        List<PhotonPipelineResult> results = results();
        Optional<EstimatedRobotPose> estimation = Optional.empty();

        for (PhotonPipelineResult result : results) {
            estimation = estimator.update(result);
            updateStdDevs(estimation, result.getTargets());
        }

        estimation.ifPresent(estimatedRobotPose -> {
            data.measurement = new VisionMeasurement(
                estimatedRobotPose.estimatedPose,
                stdDevs,
                estimatedRobotPose.timestampSeconds
            );
        });
        latestResult = results.isEmpty() ? null : results.get(0);
    }

    void updateStdDevs(Optional<EstimatedRobotPose> estimation, List<PhotonTrackedTarget> targets) {
        if (estimation.isEmpty()) {
            stdDevs = VisionConstants.StdDevs.kSingleTag;
        } else {
            Matrix<N3, N1> devsEst = VisionConstants.StdDevs.kSingleTag;
            int numTags = 0;
            double avgDist = 0;

            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;

                numTags++;
                double dist = tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimation.get().estimatedPose.toPose2d().getTranslation());

                avgDist += dist;
            }

            if (numTags == 0) {
                stdDevs = VisionConstants.StdDevs.kSingleTag;
            } else {
                avgDist /= numTags;
                double untrusted = Double.MAX_VALUE;
                if (numTags > 1) devsEst = VisionConstants.StdDevs.kMultiTag;
                if (numTags == 1 && avgDist > 4) devsEst = VecBuilder.fill(untrusted, untrusted, untrusted);
                else devsEst = devsEst.times(1 + (avgDist * avgDist / 30));
                stdDevs = devsEst;
            }
        }
    }

    /**
     * Get the latest result from the camera. May be null
     *
     * @return the latest result from the camera
     */
    public PhotonPipelineResult getLatestResult() { return latestResult; }
}
