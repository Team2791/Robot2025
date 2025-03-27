package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import frc.robot.event.EventRegistry;

import java.util.function.BiFunction;
import java.util.function.Consumer;

/**
 * Photon camera subsystem, owns cameras and handles pose estimation.
 * Does not extend SubsystemBase because this should not be used as a command requirement!
 */
public class Photon {
    final Consumer<CameraIO.VisionMeasurement> measurementConsumer;

    final CameraIO front;
    final CameraIO rear;
    final CameraIO driver;

    public Photon(
        Consumer<CameraIO.VisionMeasurement> measurementConsumer,
        BiFunction<String, Transform3d, CameraIO> cameraFactory
    ) {
        this.measurementConsumer = measurementConsumer;
        this.front = cameraFactory.apply(VisionConstants.Names.kFront, VisionConstants.Transforms.kFront);
        this.rear = cameraFactory.apply(VisionConstants.Names.kRear, VisionConstants.Transforms.kRear);
        this.driver = cameraFactory.apply(VisionConstants.Names.kDriver, new Transform3d());
        this.driver.setDriverMode(true);

        EventRegistry.periodic.register(this::periodic);
    }

    public void periodic() {
        // update cameras
        front.update();
        rear.update();

        // add to logger
        //        Logger.processInputs("Photon/Front", front.data);
        //        Logger.processInputs("Photon/Rear", rear.data);

        // make vision odometry measurements
        if (front.data.measurement != null && front.nearestTarget() <= VisionConstants.kMaxDistance)
            measurementConsumer.accept(front.data.measurement);

        if (rear.data.measurement != null && rear.nearestTarget() <= VisionConstants.kMaxDistance)
            measurementConsumer.accept(rear.data.measurement);
    }
}
