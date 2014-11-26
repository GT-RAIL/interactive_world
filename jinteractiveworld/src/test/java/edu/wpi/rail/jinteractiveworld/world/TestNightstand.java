package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestNightstand {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Nightstand.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Nightstand.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Nightstand.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Nightstand n = new Nightstand();
		assertEquals((String) f1.get(n), n.getName());
		assertEquals((double) f2.get(n), n.getWidth());
		assertEquals((double) f3.get(n), n.getHeight());
	}
}
