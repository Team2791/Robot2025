package frc.robot.subsystems.lift;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DispenserConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robotio.lift.DispenserIO;
import frc.robotio.lift.DispenserIO.DispenserData;
import frc.robotio.lift.ElevatorIO;
import frc.robotio.lift.ElevatorIO.ElevatorData;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Meters;

public class Lift extends SubsystemBase {
    public static class ReefRange extends Emitter.Event<Double> {
        @Override
        public Emitter.Dependency<Double, Pose2d> runAfter() {
            final AprilTagFieldLayout layout = VisionConstants.kField;

            final List<Integer> blue = List.of(17, 18, 19, 20, 21, 22);
            final Stream<AprilTag> blueTags = layout.getTags().stream().filter(tag -> blue.contains(tag.ID));
            final Stream<Translation2d> blueSides = blueTags.map(tag -> tag.pose.toPose2d().getTranslation());
            final Translation2d blueAvg = blueSides.reduce(Translation2d::plus).orElse(new Translation2d()).div(6);

            final List<Integer> red = List.of(6, 7, 8, 9, 10, 11);
            final Stream<AprilTag> redTags = layout.getTags().stream().filter(tag -> red.contains(tag.ID));
            final Stream<Translation2d> redSides = redTags.map(tag -> tag.pose.toPose2d().getTranslation());
            final Translation2d redAvg = redSides.reduce(Translation2d::plus).orElse(new Translation2d()).div(6);

            return new Emitter.Dependency<>(
                new Drivetrain.PoseResetEvent(),
                (pose) -> {
                    Translation2d robot = pose.getTranslation();
                    double dx2 = Math.pow(blueAvg.getX() - robot.getX(), 2);
                    double dy2 = Math.pow(blueAvg.getY() - robot.getY(), 2);
                    double blueDist2 = dx2 + dy2;

                    dx2 = Math.pow(redAvg.getX() - robot.getX(), 2);
                    dy2 = Math.pow(redAvg.getY() - robot.getY(), 2);
                    double redDist2 = dx2 + dy2;

                    return Math.sqrt(Math.min(blueDist2, redDist2));
                }
            );
        }
    }

    final DispenserIO dispenser;
    final ElevatorIO elevator;

    public Lift(DispenserIO dispenser, ElevatorIO elevator) {
        this.dispenser = dispenser;
        this.elevator = elevator;
    }

    /** Level is 0 for intake, [1, 4] for [L1, L4] scoring */
    public void elevate(int level) {
        assert level >= 0 && level <= 4 : "Invalid level: " + level + " (expected [0, 4])";
        double height = ElevatorConstants.Heights.kLevels[level];
        elevator.setDesiredPosition(Meters.of(height));
    }

    /** Return whether we are >= L1 (within tolerance) */
    public boolean canDispense() {
        return getLevel() >= 1 - ElevatorConstants.Heights.kLevelTolerance;
    }

    /** Outputs [0, 4], where 0 is intake, and decimals are in-between */
    @AutoLogOutput(key = "Lift/Elevator/Level")
    public double getLevel() {
        double height = elevator.data.height().in(Meters);
        for (int i = 0; i < ElevatorConstants.Heights.kLevels.length - 1; i++) {
            if (height < ElevatorConstants.Heights.kLevels[i + 1]) {
                double relative = height - ElevatorConstants.Heights.kLevels[i];
                double range = ElevatorConstants.Heights.kLevels[i + 1] - ElevatorConstants.Heights.kLevels[i];
                return i + relative / range;
            }
        }
        return 4;
    }

    /** Check if at level */
    public boolean atLevel(int level) {
        assert level >= 0 && level <= 4 : "Invalid level: " + level + " (expected [0, 4])";
        double height = elevator.data.height().in(Meters);
        double levelHeight = ElevatorConstants.Heights.kLevels[level];
        double error = Math.abs(height - levelHeight);
        return error < ElevatorConstants.Heights.kTolerance;
    }

    /** Run the dispenser */
    public void dispense() {
        dispense(DispenserConstants.kDispense);
    }

    /** Run the dispenser (with power) */
    public void dispense(double power) {
        assert Math.abs(power) <= 1.0 : "Needed -1.0 <= power <= 1.0, got %f".formatted(power);
        dispenser.set(power);
    }

    /** Get a copy of the dispenser data */
    public DispenserData getDispenser() {
        return dispenser.data.clone();
    }

    /** Get a copy of the elevator data */
    public ElevatorData getElevator() {
        return elevator.data.clone();
    }

    @Override
    public void periodic() {
        // update the elevator and dispenser
        elevator.update();
        dispenser.update();

        // log
        Logger.processInputs("Lift/Dispenser", dispenser.data);
        Logger.processInputs("Lift/Elevator", elevator.data);
    }
}
