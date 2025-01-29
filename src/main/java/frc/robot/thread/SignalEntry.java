package frc.robot.thread;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public class SignalEntry<T> {
	final Supplier<T> signal;
	final Queue<T> cache;

	SignalEntry(Supplier<T> signal) {
		this.signal = signal;
		this.cache = new ArrayBlockingQueue<>(20);
	}

	void update() {
		cache.offer(signal.get());
	}
}
