package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.List;

public class StationAlign extends TagAlign {
    /**
     * Align to the station using photon vision, positioning self for intaking
     *
     * @param drivetrain the drivetrain subsystem
     */
    public StationAlign(Drivetrain drivetrain) {
        super(
            drivetrain,
            new Transform2d(
                0.5 * RobotConstants.DriveBase.kBumperLength,
                0,
                new Rotation2d()
            )
        );
    }

    @Override
    public List<Integer> getTagIds() {
        return VisionConstants.AprilTag.stations();
    }
}
