package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestBurner {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Burner.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Burner.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Burner.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Burner b = new Burner();
		assertEquals((String) f1.get(b), b.getName());
		assertEquals((double) f2.get(b), b.getWidth());
		assertEquals((double) f3.get(b), b.getHeight());
	}
}
