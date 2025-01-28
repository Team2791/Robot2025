package frc.robot.thread;

import java.util.Collection;
import java.util.Iterator;
import java.util.Queue;

import frc.robot.util.Timestamped;

public class TimestampedQueue<E> implements Queue<Timestamped<E>> {
	final Queue<E> cache;
	final Queue<Double> timestamps;

	TimestampedQueue(Queue<E> cache, Queue<Double> timestamps) {
		assert cache.size() == timestamps.size() : "Cache and timestamps must be the same size";

		this.cache = cache;
		this.timestamps = timestamps;
	}


	/** Interface methods, standard stuff */
	@Override
	public boolean add(Timestamped<E> e) {
		cache.add(e.value());
		timestamps.add(e.timestamp());
		return true;
	}

	@Override
	public boolean offer(Timestamped<E> e) {
		cache.offer(e.value());
		timestamps.offer(e.timestamp());
		return true;
	}

	@Override
	public Timestamped<E> remove() {
		return new Timestamped<>(cache.remove(), timestamps.remove());
	}

	@Override
	public Timestamped<E> poll() {
		return new Timestamped<>(cache.poll(), timestamps.poll());
	}

	@Override
	public Timestamped<E> element() {
		return new Timestamped<>(cache.element(), timestamps.element());
	}

	@Override
	public Timestamped<E> peek() {
		return new Timestamped<>(cache.peek(), timestamps.peek());
	}

	@Override
	public int size() {
		return cache.size();
	}

	@Override
	public boolean isEmpty() { return cache.isEmpty(); }

	@Override
	public boolean contains(Object o) {
		return cache.contains(o);
	}

	@Override
	public Object[] toArray() {
		return cache.toArray();
	}

	@Override
	public <T> T[] toArray(T[] a) {
		return cache.toArray(a);
	}

	@Override
	public boolean remove(Object o) {
		return cache.remove(o);
	}

	@Override
	public boolean containsAll(Collection<?> c) {
		return cache.containsAll(c);
	}

	@Override
	public boolean addAll(Collection<? extends Timestamped<E>> c) {
		for (Timestamped<E> e : c) {
			add(e);
		}
		return true;
	}

	@Override
	public boolean removeAll(Collection<?> c) {
		return cache.removeAll(c);
	}

	@Override
	public boolean retainAll(Collection<?> c) {
		return cache.retainAll(c);
	}

	@Override
	public void clear() {
		cache.clear();
		timestamps.clear();
	}

	@Override
	public boolean equals(Object o) {
		return cache.equals(o);
	}

	@Override
	public int hashCode() {
		return cache.hashCode();
	}

	@Override
	public Iterator<Timestamped<E>> iterator() {
		return new Iterator<Timestamped<E>>() {
			Iterator<E> cacheIterator = cache.iterator();
			Iterator<Double> timestampIterator = timestamps.iterator();

			@Override
			public boolean hasNext() {
				return cacheIterator.hasNext();
			}

			@Override
			public Timestamped<E> next() {
				return new Timestamped<>(cacheIterator.next(), timestampIterator.next());
			}
		};
	}
}
