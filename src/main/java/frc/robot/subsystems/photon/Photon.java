package frc.robot.subsystems.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robotio.photon.CameraIO;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.stream.Stream;

public class Photon extends SubsystemBase {
    final Drivetrain drivetrain;

    final CameraIO front;

    public Photon(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.front = new Camera(VisionConstants.Names.kFront);
    }

    public CameraIO.CameraData getFrontData() {
        return front.data.clone();
    }

    void processResults(PhotonPipelineResult[] results, Transform3d bot2cam) {
        final AprilTagFieldLayout field = VisionConstants.kField;

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            Pose3d bot;
            Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();

            if (multiTagResult.isEmpty()) {
                // result has at least one target
                Stream<PhotonTrackedTarget> withTags = result.getTargets().stream().filter(t -> t.fiducialId != -1);
                Optional<PhotonTrackedTarget> target = withTags.findFirst();
                Optional<Pose3d> targetPose = target.flatMap(t -> field.getTagPose(t.fiducialId));

                if (target.isEmpty() || targetPose.isEmpty()) continue;

                Transform3d cam2target = target.get().bestCameraToTarget;
                Transform3d bot2target = bot2cam.plus(cam2target);
                bot = targetPose.get().transformBy(bot2target.inverse());
            } else {
                Transform3d field2cam = multiTagResult.get().estimatedPose.best;
                Transform3d field2bot = field2cam.plus(bot2cam.inverse());
                bot = new Pose3d(field2bot.getTranslation(), field2bot.getRotation());
            }

            drivetrain.addVisionMeasurement(bot.toPose2d(), result.getTimestampSeconds());
        }
    }

    @Override
    public void periodic() {
        // update cameras
        front.update();

        // output to logger
        Logger.processInputs("Photon/Front", front.data);

        // make vision odometry measurements
        processResults(front.data.results, VisionConstants.Transforms.kBotToFront);
    }
}
