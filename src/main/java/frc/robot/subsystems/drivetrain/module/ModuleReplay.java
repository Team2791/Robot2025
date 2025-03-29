package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.constants.ModuleConstants;

public class ModuleReplay extends ModuleIO {
    public ModuleReplay(ModuleConstants.ModuleInfo id) {
        super(id);
    }

    @Override
    public void update() { }

    @Override
    public void setIdleMode(SparkBaseConfig.IdleMode mode) { }

    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) { }

    @Override
    public void driveOpenLoop(double output) { }

    @Override
    public void zeroTurn() { }
}
