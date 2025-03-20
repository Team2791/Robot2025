package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import frc.robot.event.EventRegistry;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.BiFunction;

/**
 * Photon camera subsystem, owns cameras and handles pose estimation.
 * Does not extend SubsystemBase because this should not be used as a command requirement!
 */
public class Photon {
    final Drivetrain drivetrain;

    final CameraIO front;
    final CameraIO rear;
    // final CameraIO orpheus;

    public Photon(Drivetrain drivetrain, BiFunction<String, Transform3d, CameraIO> cameraFactory) {
        this.drivetrain = drivetrain;
        this.front = cameraFactory.apply(VisionConstants.Names.kFront, VisionConstants.Transforms.kFront);
        this.rear = cameraFactory.apply(VisionConstants.Names.kRear, VisionConstants.Transforms.kRear);
        // this.orpheus = cameraFactory.apply(VisionConstants.Names.kOrpheus, VisionConstants.Transforms.kOrpheus);

        EventRegistry.periodic.register(this::periodic);
    }

    public void periodic() {
        // update cameras
        front.update();
        rear.update();
        // orpheus.update();

        // add to logger
        Logger.processInputs("Photon/Front", front.data);
        Logger.processInputs("Photon/Rear", rear.data);
        // Logger.processInputs("Photon/Orpheus", orpheus.data);

        // make vision odometry measurements
        if (front.data.measurement != null) drivetrain.addVisionMeasurement(front.data.measurement);
        if (rear.data.measurement != null) drivetrain.addVisionMeasurement(rear.data.measurement);
        // if (orpheus.data.measurement != null) drivetrain.addVisionMeasurement(orpheus.data.measurement);
    }
}
