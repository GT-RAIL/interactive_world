package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestOven {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Oven.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Oven.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Oven.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Oven o = new Oven();
		assertEquals((String) f1.get(o), o.getName());
		assertEquals((double) f2.get(o), o.getWidth());
		assertEquals((double) f3.get(o), o.getHeight());
	}
}
