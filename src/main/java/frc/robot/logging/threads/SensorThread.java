package frc.robot.logging.threads;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.ThreadConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

@SuppressWarnings("rawtypes")
public class SensorThread {
    private static SensorThread instance;
    private final Notifier scheduler;

    private final List<SignalEntry> entries;
    private final ArrayList<Queue<Double>> timestamps;
    private final ReadWriteLock lock = new ReentrantReadWriteLock();

    private SensorThread() {
        this.entries = new ArrayList<>();
        this.timestamps = new ArrayList<>();
        this.scheduler = new Notifier(this::run);

        scheduler.setName("SensorThread");
        scheduler.startPeriodic(ThreadConstants.kDelay);
    }

    public static SensorThread getInstance() {
        if (instance == null) {
            instance = new SensorThread();
        }
        return instance;
    }

    public <T> Queue<T> register(Supplier<T> signal) {
        SignalEntry<T> entry = new SignalEntry<>(signal);

        lock.writeLock().lock(); // prevent reads while we write
        entries.add(entry);
        lock.writeLock().lock();

        return entry.cache;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> timestamps = new ArrayBlockingQueue<>(ThreadConstants.kCacheSize);
        this.timestamps.add(timestamps);
        return timestamps;
    }

    private void run() {
        double ts = RobotController.getFPGATime() / 1e6;

        lock.readLock().lock(); // prevent writes while we read
        for (int i = 0; i < entries.size(); i++) {
            entries.get(i).update();

            if (i < timestamps.size()) timestamps.get(i).offer(ts);
        }
        lock.readLock().unlock();
    }

    public void stop() {
        scheduler.stop();
    }
}
