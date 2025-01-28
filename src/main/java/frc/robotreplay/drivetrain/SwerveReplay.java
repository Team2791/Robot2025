package frc.robotreplay.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robotio.drivetrain.SwerveIO;

public class SwerveReplay extends SwerveIO {
	public SwerveReplay(int driveId, int turnId, double angularOffset) {
		super(driveId, turnId, angularOffset);
	}

	public void update() {}

	public void setDesiredState(SwerveModuleState desired) {}
}
