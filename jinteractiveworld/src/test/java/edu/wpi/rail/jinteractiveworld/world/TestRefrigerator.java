package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestRefrigerator {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Refrigerator.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Refrigerator.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Refrigerator.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Refrigerator r = new Refrigerator();
		assertEquals((String) f1.get(r), r.getName());
		assertEquals((double) f2.get(r), r.getWidth());
		assertEquals((double) f3.get(r), r.getHeight());
	}
}
