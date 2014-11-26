package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestPlate {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Plate.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Plate.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Plate.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Plate p = new Plate();
		assertEquals((String) f1.get(p), p.getName());
		assertEquals((double) f2.get(p), p.getWidth());
		assertEquals((double) f3.get(p), p.getHeight());
	}
}
