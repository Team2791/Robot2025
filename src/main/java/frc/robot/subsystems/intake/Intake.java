package frc.robot.subsystems.intake;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robotio.RollerIO;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

public class Intake extends SubsystemBase {
    public static class IntakeRange extends Emitter.Event<Double> {
        @Override
        public Emitter.Dependency<Double, Pose2d> runAfter() {
            final AprilTagFieldLayout layout = VisionConstants.AprilTag.kLayout;
            final List<Translation2d> stations = VisionConstants.AprilTag.stations()
                .stream()
                .map(layout::getTagPose)
                .filter(Optional::isPresent)
                .map(p -> p.get().toPose2d().getTranslation())
                .toList();

            return new Emitter.Dependency<>(
                new Drivetrain.PoseUpdateEvent(),
                (pose) -> {
                    Translation2d robot = pose.getTranslation();
                    double nearest2 = Double.MAX_VALUE;

                    for (Translation2d station : stations) {
                        double dx2 = Math.pow(station.getX() - robot.getX(), 2);
                        double dy2 = Math.pow(station.getY() - robot.getY(), 2);
                        double dist2 = dx2 + dy2;
                        if (dist2 < nearest2) nearest2 = dist2;
                    }

                    return Math.sqrt(nearest2);
                }
            );
        }
    }

    final RollerIO roller;

    public Intake(RollerIO roller) {
        this.roller = roller;
    }

    public RollerIO.RollerData getRoller() {
        return roller.data;
    }

    public void set(double left, double right) {
        roller.set(left, right);
    }

    public void set(double power) {
        set(power, -power);
    }

    public void stop() {
        set(0, 0);
    }

    @Override
    public void periodic() {
        roller.update();

        Logger.processInputs("Intake", roller.data);
    }
}
