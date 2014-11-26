package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestTV {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = TV.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = TV.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = TV.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		TV t = new TV();
		assertEquals((String) f1.get(t), t.getName());
		assertEquals((double) f2.get(t), t.getWidth());
		assertEquals((double) f3.get(t), t.getHeight());
	}
}
