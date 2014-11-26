package edu.wpi.rail.jinteractiveworld.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestObject {

	@Test
	public void testConstructor() {
		String name = "test";
		double width = 0.5;
		double height = 0.25;
		Object o = new DummyObject(name, width, height);
		assertEquals(name, o.getName());
		assertEquals(width, o.getWidth());
		assertEquals(height, o.getHeight());
	}

	@Test
	public void testSetName() {
		String name1 = "test1";
		Object o = new DummyObject(name1, 0.5, 0.25);
		assertEquals(name1, o.getName());
		String name2 = "test2";
		o.setName(name2);
		assertEquals(name2, o.getName());
	}

	@Test
	public void testSetWidth() {
		double width1 = 0.5;
		Object o = new DummyObject("test", width1, 0.25);
		assertEquals(width1, o.getWidth());
		double width2 = 1.5;
		o.setWidth(width2);
		assertEquals(width2, o.getWidth());
	}

	@Test
	public void testSetHeight() {
		double height1 = 0.25;
		Object o = new DummyObject("test", 0.5, height1);
		assertEquals(height1, o.getHeight());
		double height2 = 1.25;
		o.setHeight(height2);
		assertEquals(height2, o.getHeight());
	}

	@Test
	public void testHashCode() {
		String name = "test";
		Object o = new DummyObject(name, 0.5, 0.25);
		;
		assertEquals(name.hashCode(), o.hashCode());
	}

	@Test
	public void testEqualsSameObject() {
		Object o = new DummyObject("test", 0.5, 0.25);
		assertTrue(o.equals(o));
	}

	@Test
	public void testEqualsSameName() {
		String name = "test";
		Object o1 = new DummyObject(name, 0.5, 0.25);
		Object o2 = new DummyObject(name, 1.5, 1.25);
		assertTrue(o1.equals(o2));
	}

	@Test
	public void testEqualsWrongType() {
		String name = "test";
		Object o = new DummyObject(name, 0.5, 0.25);
		assertFalse(o.equals(name));
	}

	@Test
	public void testEqualsWrongName() {
		String name1 = "test1";
		String name2 = "test2";
		Object o1 = new DummyObject(name1, 0.5, 0.25);
		Object o2 = new DummyObject(name2, 1.5, 1.25);
		assertFalse(o1.equals(o2));
	}

	private class DummyObject extends Object {

		public DummyObject(String name, double width, double height) {
			super(name, width, height);
		}
	}
}