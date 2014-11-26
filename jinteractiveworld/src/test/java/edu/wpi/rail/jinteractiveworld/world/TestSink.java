package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestSink {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Sink.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Sink.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Sink.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Sink s = new Sink();
		assertEquals((String) f1.get(s), s.getName());
		assertEquals((double) f2.get(s), s.getWidth());
		assertEquals((double) f3.get(s), s.getHeight());
	}
}
