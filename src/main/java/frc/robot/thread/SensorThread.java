package frc.robot.thread;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.constants.SignalConstants;

public class SensorThread {
	@SuppressWarnings("rawtypes")
	private final List<SignalEntry> entries;
	private final ArrayList<Queue<Double>> timestamps;

	/** ArrayList is not threadsafe. Need to lock on read-write */
	private final ReadWriteLock lock = new ReentrantReadWriteLock();

	private static SensorThread instance;
	private Notifier scheduler;

	public static SensorThread getInstance() {
		if (instance == null) {
			instance = new SensorThread();
		}
		return instance;
	}

	private SensorThread() {
		this.entries = new ArrayList<>();
		this.timestamps = new ArrayList<>(20);
		this.scheduler = new Notifier(this::run);

		scheduler.setName("SensorThread");
		scheduler.startPeriodic(SignalConstants.kDelay);
	}

	public <T> Queue<T> register(Supplier<T> signal) {
		SignalEntry<T> entry = new SignalEntry<>(signal);

		lock.writeLock().lock(); // prevent reads while we write
		entries.add(entry);
		lock.writeLock().lock();

		return entry.cache;
	}

	public Queue<Double> makeTimestampQueue() {
		Queue<Double> timestamps = new ArrayBlockingQueue<>(20);
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
}
