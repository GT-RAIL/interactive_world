package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import java.lang.reflect.Field;

import org.junit.Test;

public class TestDiningTable {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = DiningTable.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = DiningTable.class.getDeclaredField("WIDTH");
		f2.setAccessible(true);
		Field f3 = DiningTable.class.getDeclaredField("HEIGHT");
		f3.setAccessible(true);
		DiningTable t = new DiningTable();
		assertEquals((String) f1.get(t), t.getName());
		assertEquals((double) f2.get(t), t.getWidth());
		assertEquals((double) f3.get(t), t.getHeight());
	}
}
