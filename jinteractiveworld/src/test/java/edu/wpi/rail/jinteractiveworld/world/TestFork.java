package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestFork {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = Fork.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = Fork.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = Fork.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		Fork fork = new Fork();
		assertEquals((String) f1.get(fork), fork.getName());
		assertEquals((double) f2.get(fork), fork.getWidth());
		assertEquals((double) f3.get(fork), fork.getHeight());
	}
}
