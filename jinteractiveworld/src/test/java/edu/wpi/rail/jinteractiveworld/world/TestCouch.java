package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestCouch {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Couch.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Couch.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Couch.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Couch c = new Couch();
		assertEquals((String) f1.get(c), c.getName());
		assertEquals((double) f2.get(c), c.getWidth());
		assertEquals((double) f3.get(c), c.getHeight());
	}
}
