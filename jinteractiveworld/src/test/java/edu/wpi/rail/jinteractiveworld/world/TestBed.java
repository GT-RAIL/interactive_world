package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestBed {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Bed.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Bed.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Bed.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Bed b = new Bed();
		assertEquals((String) f1.get(b), b.getName());
		assertEquals((double) f2.get(b), b.getWidth());
		assertEquals((double) f3.get(b), b.getHeight());
	}
}
