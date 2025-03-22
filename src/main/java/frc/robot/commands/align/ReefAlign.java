package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;

public class ReefAlign extends TagAlign {
    /**
     * Align to the reef using photon vision, positioning self to score on a particular branch
     *
     * @param drivetrain the drivetrain subsystem
     * @param direction  the direction to align to. -1 for left, 0 for center, 1 for right
     */
    public ReefAlign(Drivetrain drivetrain, int direction) {
        super(
            drivetrain,
            new Transform2d(
                0.5 * RobotConstants.DriveBase.kBumperLength - 0.05,
                VisionConstants.Align.kReefOffset * direction,
                Rotation2d.kPi
            )
        );
    }

    @Override
    protected List<Integer> getTagIds() {
        return VisionConstants.AprilTag.reef();
    }
}
