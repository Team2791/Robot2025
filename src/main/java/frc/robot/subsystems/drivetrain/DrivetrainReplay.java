package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DrivetrainReplay extends DrivetrainIO {
    private final Field2d field = new Field2d();

    @Override
    public void update() {}

    @Override
    public Field2d getField2d() {
        return this.field;
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode mode) {}
    
    @Override
    public void resetPose(Pose2d pose) {}
}
