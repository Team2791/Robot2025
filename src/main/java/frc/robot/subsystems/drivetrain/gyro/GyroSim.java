package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroSim extends GyroIO {
    final GyroSimulation sim;

    public GyroSim(GyroSimulation sim) {
        this.sim = sim;
    }

    @Override
    public void update() {
        data.heading = sim.getGyroReading().getMeasure();
        data.velocity = sim.getMeasuredAngularVelocity();
        data.connected = false;
    }

    @Override
    public void reset(Rotation2d rotation) {
        sim.setRotation(rotation);
    }
}
