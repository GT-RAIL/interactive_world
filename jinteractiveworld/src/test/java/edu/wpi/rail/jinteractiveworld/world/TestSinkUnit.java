package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestSinkUnit {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = SinkUnit.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = SinkUnit.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = SinkUnit.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		SinkUnit s = new SinkUnit();
		assertEquals((String) f1.get(s), s.getName());
		assertEquals((double) f2.get(s), s.getWidth());
		assertEquals((double) f3.get(s), s.getHeight());
	}
}
