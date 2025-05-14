package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.BuildConstants;
import frc.robot.event.EventRegistry;
import frc.robot.util.ADStar;
import frc.robot.util.Elastic;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.util.Date;

public class Robot extends LoggedRobot {
    final RobotContainer container;

    Command autoCommand;

    public Robot() {
        // setup logger constants.
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

        // setup logger data receivers, i.e. where logs are kept when they are made
        switch (AdvantageConstants.kCurrentMode) {
            case Real:
                String date = new Date().toString().replaceAll(" ", "_").replaceAll(":", "-"); // imagine windows
                String log = String.format("/U/logs/akit_%s_%s.wpilog", date, BuildConstants.GIT_SHA);

                Logger.addDataReceiver(new WPILOGWriter(log));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case Sim:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case Replay:
                String logfile = LogFileUtil.findReplayLog();
                setUseTiming(false); // make it go fast
                Logger.setReplaySource(new WPILOGReader(logfile));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logfile, ".sim")));
                break;
        }

        // register rev hardware logger
        Logger.registerURCL(URCL.startExternal());

        // start logger
        Logger.start();

        // elastic remote downloading thing
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // setup everything else
        this.container = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // setup pp pathfinder (currently not being used, we'll see later)
        Pathfinding.setPathfinder(new ADStar());
        FollowPathCommand.warmupCommand().schedule();

        // give drivers the auto tab at the beginning of the game
        Elastic.selectTab("Autonomous");
    }

    @Override
    public void robotPeriodic() {
        // Performance: give ourselves very high priority
        Threads.setCurrentThreadPriority(true, 99);

        // Run the robot for a tick
        CommandScheduler.getInstance().run();

        // Run event emitter periodic event
        EventRegistry.periodic.emit();

        // High prio no longer needed
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledPeriodic() { }

    /** Find and schedule the autonomous command */
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
            CommandScheduler.getInstance().cancelAll();
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
