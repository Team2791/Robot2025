package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.function.BiFunction;

/**
 * Photon camera subsystem, owns cameras and handles pose estimation.
 * Does not extend SubsystemBase because this should not be used as a command requirement!
 */
public class Photon {
    final Drivetrain drivetrain;
    final CameraIO front;

    public Photon(Drivetrain drivetrain, BiFunction<String, Transform3d, CameraIO> cameraFactory) {
        this.drivetrain = drivetrain;
        this.front = cameraFactory.apply(VisionConstants.Names.kFront, VisionConstants.Transforms.kBotToFront);

        Emitter.on(new Robot.PeriodicEvent(), _mode -> this.periodic());
    }

    public PhotonPipelineResult frontResult() {
        return front.getLatestResult();
    }

    public void periodic() {
        // update cameras
        front.update();

        // add to logger
        Logger.processInputs("Photon/Front", front.data);

        // make vision odometry measurements
        if (front.data.measurement != null) drivetrain.addVisionMeasurement(front.data.measurement);
    }
}
