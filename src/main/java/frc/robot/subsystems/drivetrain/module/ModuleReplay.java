package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleReplay extends ModuleIO {
    /** Keep constructors the same so I can use fn pointers without java screaming */
    public ModuleReplay(int _id) { }

    @Override
    public void update() { }

    @Override
    public void setIdleMode(SparkBaseConfig.IdleMode mode) { }

    @Override
    public void setDesiredState(SwerveModuleState desired) { }
}
