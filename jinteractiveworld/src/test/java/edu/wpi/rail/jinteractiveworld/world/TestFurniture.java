package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestFurniture {

	@Test
	public void testConstructor() {
		String name = "test";
		double width = 0.5;
		double height = 0.25;
		Furniture f = new DummyFurniture(name, width, height);
		assertEquals(name, f.getName());
		assertEquals(width, f.getWidth());
		assertEquals(height, f.getHeight());
	}

	private class DummyFurniture extends Furniture {

		public DummyFurniture(String name, double width, double height) {
			super(name, width, height);
		}
	}
}
