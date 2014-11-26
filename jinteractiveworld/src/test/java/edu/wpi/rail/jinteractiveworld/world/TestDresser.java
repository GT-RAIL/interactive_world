package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestDresser {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Dresser.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Dresser.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Dresser.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Dresser d = new Dresser();
		assertEquals((String) f1.get(d), d.getName());
		assertEquals((double) f2.get(d), d.getWidth());
		assertEquals((double) f3.get(d), d.getHeight());
	}
}
