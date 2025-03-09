package frc.robot.subsystems.dispenser;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DispenserConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.event.Event;
import frc.robot.event.EventDependency;
import frc.robot.subsystems.dispenser.DispenserIO.DispenserData;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Dispenser extends SubsystemBase {
    public static class ReefRange extends Event<Double> {
        @Override
        public EventDependency<Double, Pose2d> runAfter() {
            final AprilTagFieldLayout layout = VisionConstants.AprilTag.kLayout;

            final Translation2d avg = VisionConstants.AprilTag.reef() // get current-alliance reef
                .stream()
                .map(layout::getTagPose) // map tag id to pose
                .filter(Optional::isPresent)
                .map(p -> p.get().toPose2d().getTranslation()) // get translation and average
                .reduce(Translation2d::plus)
                .orElse(new Translation2d())
                .div(6);

            return new EventDependency<>(
                new Drivetrain.PoseUpdateEvent(),
                pose -> pose.getTranslation().getDistance(avg)
            );
        }
    }

    final DispenserIO dispenser;

    public Dispenser(DispenserIO dispenser) {
        this.dispenser = dispenser;
    }

    /** Run the dispenser */
    public void dispense() {
        dispense(DispenserConstants.Power.kDispense);
    }

    /** Run the dispenser (with power) */
    public void dispense(double power) {
        assert Math.abs(power) <= 1.0 : "Needed -1.0 <= power <= 1.0, got %f".formatted(power);
        dispenser.set(power);
    }

    /** Get a copy of the dispenser data */
    public DispenserData data() {
        return dispenser.data.clone();
    }

    @Override
    public void periodic() {
        dispenser.update();
        Logger.processInputs("Dispenser", dispenser.data);
    }
}
