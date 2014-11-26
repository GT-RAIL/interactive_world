package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestPillow {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Pillow.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Pillow.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Pillow.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Pillow p = new Pillow();
		assertEquals((String) f1.get(p), p.getName());
		assertEquals((double) f2.get(p), p.getWidth());
		assertEquals((double) f3.get(p), p.getHeight());
	}
}
