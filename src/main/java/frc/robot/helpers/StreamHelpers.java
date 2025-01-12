package frc.robot.helpers;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

public class StreamHelpers {
	private StreamHelpers() {}

	/**
	 * Zips two streams together, applying a consumer to each pair of elements.
	 * 
	 * @param <T>      The type of the first stream
	 * @param <K>      The type of the second stream
	 * @param a        The first stream
	 * @param b        The second stream
	 * @param consumer The consumer to apply to each pair of elements
	 */
	public static <T, K> void zipThen(Stream<T> a, Stream<K> b, BiConsumer<T, K> consumer) {
		Iterator<T> aIterator = a.iterator();
		Iterator<K> bIterator = b.iterator();
		while (aIterator.hasNext() && bIterator.hasNext()) {
			consumer.accept(aIterator.next(), bIterator.next());
		}
	}

	/**
	 * Zips two streams together, returning a stream of pairs of elements.
	 * 
	 * @param <T> The type of the first stream
	 * @param <K> The type of the second stream
	 * @param a   The first stream
	 * @param b   The second stream
	 * @return A stream of pairs of elements
	 */
	public static <T, K> Stream<Entry<T, K>> zip(Stream<T> a, Stream<K> b) {
		Iterator<T> aIterator = a.iterator();
		Iterator<K> bIterator = b.iterator();
		return Stream.iterate(
			true,
			i -> aIterator.hasNext() && bIterator.hasNext(),
			i -> i
		)
			.map(i -> Map.entry(aIterator.next(), bIterator.next()));
	}
}
