package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestMagazines {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Magazines.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Magazines.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Magazines.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Magazines m = new Magazines();
		assertEquals((String) f1.get(m), m.getName());
		assertEquals((double) f2.get(m), m.getWidth());
		assertEquals((double) f3.get(m), m.getHeight());
	}
}
