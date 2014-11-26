package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;

public class TestVector3 {

	@Test
	public void testNoArgumentConstructor() {
		Vector3 p = new Vector3();
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testXConstructor() {
		Vector3 p = new Vector3(0.5);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testXYConstructor() {
		Vector3 p = new Vector3(0.5, 1.5);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testXYZConstructor() {
		Vector3 p = new Vector3(0.5, 1.5, 4.5);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(4.5, p.getZ(), 0);
	}

	@Test
	public void testSetX() {
		Vector3 p = new Vector3();
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setX(0.5);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testSetY() {
		Vector3 p = new Vector3();
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setY(1.5);
		assertEquals(0, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testSetZ() {
		Vector3 p = new Vector3();
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setZ(4.5);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(4.5, p.getZ(), 0);
	}

	@Test
	public void testEqualsSameObject() {
		Vector3 p = new Vector3();
		assertTrue(p.equals(p));
	}

	@Test
	public void testEqualsSameValues() {
		double x = 0.5;
		double y = 1.5;
		double z = 4.5;
		Vector3 v1 = new Vector3(x, y, z);
		Vector3 v2 = new Vector3(x, y, z);
		assertTrue(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongType() {
		Vector3 p = new Vector3();
		assertFalse(p.equals(new String()));
	}

	@Test
	public void testEqualsWrongX() {
		double y = 1.5;
		double z = 4.5;
		Vector3 v1 = new Vector3(0.5, y, z);
		Vector3 v2 = new Vector3(0, y, z);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongY() {
		double x = 0.5;
		double z = 4.5;
		Vector3 v1 = new Vector3(x, 1.5, z);
		Vector3 v2 = new Vector3(x, 0, z);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongZ() {
		double x = 0.5;
		double y = 1.5;
		Vector3 v1 = new Vector3(x, y, 4.5);
		Vector3 v2 = new Vector3(x, y, 0);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testClone() {
		Vector3 v = new Vector3(1, 2, 3);
		assertEquals(v, v.clone());
	}
}
