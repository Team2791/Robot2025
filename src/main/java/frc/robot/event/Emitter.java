package frc.robot.event;

import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

@SuppressWarnings("rawtypes")
public class Emitter {

	// java complains about Class<? extends Event<?>> idk why
	private static final HashMap<Class<? extends Event>, HashSet<Consumer<?>>> listeners = new HashMap<>();

	// manage active jobs for paralleling/concurrency/whatever
	private static final ExecutorService executor = Executors.newCachedThreadPool();

	@SuppressWarnings("unchecked")
	public static <T, E extends Event<T>> void emit(E event, T data) {
		HashSet<Consumer<?>> eventListeners = listeners.get(event.getClass());

		if (eventListeners != null) {
			for (Consumer<?> listener : eventListeners) {
				executor.execute(() -> ((Consumer<T>) listener).accept(data));
			}
		}
	}

	public static <E extends Event<Void>> void emit(E event) {
		emit(event, null);
	}

	@SuppressWarnings("unchecked")
	public static <T, E extends Event<T>> Key<T, E> on(E event, Consumer<T> listener) {
		if (!listeners.containsKey(event.getClass())) {
			EventDependency<T, Object> dep = ((EventDependency<T, Object>) event.runAfter());

			if (dep != null) {
				listeners.computeIfAbsent(dep.other.getClass(), k -> new HashSet<>())
					.add(k -> Emitter.emit(event, dep.transform.apply(k)));
			}
		}

		listeners.computeIfAbsent(event.getClass(), k -> new HashSet<>()).add(listener);

		return new Key<>(event.getClass(), listener);
	}

	public static <T, E extends Event<T>> Key<T, E> on(E event, Runnable listener) {
		return on(event, _v -> listener.run());
	}

	public static <T, E extends Event<T>> void off(Key<T, E> key) {
		HashSet<Consumer<?>> eventListeners = listeners.get(key.key);

		if (eventListeners != null) {
			eventListeners.remove(key.listener);
		}
	}

	public static <T, E extends Event<T>> boolean exists(Key<T, E> key) {
		HashSet<Consumer<?>> eventListeners = listeners.get(key.key);
		return eventListeners != null && eventListeners.contains(key.listener);
	}
}
