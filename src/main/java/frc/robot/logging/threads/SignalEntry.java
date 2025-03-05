package frc.robot.logging.threads;

import frc.robot.constants.ThreadConstants;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public class SignalEntry<T> {
    final Supplier<T> signal;
    final Queue<T> cache;

    SignalEntry(Supplier<T> signal) {
        this.signal = signal;
        this.cache = new ArrayBlockingQueue<>(ThreadConstants.kCacheSize);
    }

    void update() {
        cache.offer(signal.get());
    }
}
