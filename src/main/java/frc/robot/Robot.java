package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.ADStar;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.BuildConstants;
import frc.robot.util.Elastic;
import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;

public class Robot extends LoggedRobot {
    final RobotContainer container;

    Command autoCommand;

    public Robot() throws IOException, ParseException {
        // setup logger constants
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitStatus", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitStatus", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitStatus", "Unknown");
                break;
        }

        // setup logger data receivers
        switch (AdvantageConstants.kCurrentMode) {
            case Real:
                String log = "akit_" + BuildConstants.BUILD_DATE.replaceAll(" ", "_") + ".wpi.log";
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/.log/akit/" + log));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case Sim:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            default:
                String logfile = LogFileUtil.findReplayLog();

                setUseTiming(false);
                Logger.setReplaySource(new WPILOGReader(logfile));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logfile, ".sim")));
                break;
        }

        // allow auto logging in frc.robotio, frc.robotsim, and frc.robotreplay
        AutoLogOutputManager.addPackage("frc.robotio");
        AutoLogOutputManager.addPackage("frc.robotsim");
        AutoLogOutputManager.addPackage("frc.robotreplay");

        // register rev hardware logger
        Logger.registerURCL(URCL.startExternal());

        // start logger
        Logger.start();

        // elastic remote downloading thing
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        this.container = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new ADStar());
        //FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        // Performance: give ourselves very high priority
        Threads.setCurrentThreadPriority(true, 99);

        // Run the robot for a tick
        CommandScheduler.getInstance().run();

        // High prio no longer needed
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousInit() {
        Elastic.selectTab("Autonomous");
        autoCommand = container.getAutonomousCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleoperated");
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() { }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() { }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }
}
