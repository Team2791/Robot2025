package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ReefAlign extends AlignClosest {
    /**
     * Align to the reef using photon vision, positioning self to score on a particular branch
     *
     * @param drivetrain the drivetrain subsystem
     * @param direction  the direction to align to, can be -1 for right or 1 for left
     */
    public ReefAlign(Drivetrain drivetrain, int direction) {
        super(
            drivetrain,
            VisionConstants.AprilTag::reef,
            new Transform2d(
                0.5 * RobotConstants.Drivetrain.kBumperLength,
                VisionConstants.Align.kReefOffset * direction,
                Rotation2d.kPi
            )
        );

        assert Math.abs(direction) <= 1 : "wanted direction in (-1, 0, 1), got " + direction;
    }
}
