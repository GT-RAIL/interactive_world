package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestItem {

	@Test
	public void testConstructor() {
		String name = "test";
		double width = 0.5;
		double height = 0.25;
		Item i = new DummyItem(name, width, height);
		assertEquals(name, i.getName());
		assertEquals(width, i.getWidth());
		assertEquals(height, i.getHeight());
	}

	private class DummyItem extends Item {

		public DummyItem(String name, double width, double height) {
			super(name, width, height);
		}
	}
}
