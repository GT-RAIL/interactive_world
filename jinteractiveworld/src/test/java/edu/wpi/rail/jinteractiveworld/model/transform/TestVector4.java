package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.Vector4;

public class TestVector4 {

	@Test
	public void testNoArgumentConstructor() {
		Vector4 p = new Vector4();
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testWConstructor() {
		Vector4 p = new Vector4(10.0);
		assertEquals(10.0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testWXConstructor() {
		Vector4 p = new Vector4(10.0, 0.5);
		assertEquals(10.0, p.getW(), 0);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testWXYConstructor() {
		Vector4 p = new Vector4(10.0, 0.5, 1.5);
		assertEquals(10.0, p.getW(), 0);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testWXYZConstructor() {
		Vector4 p = new Vector4(10.0, 0.5, 1.5, 4.5);
		assertEquals(10.0, p.getW(), 0);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(4.5, p.getZ(), 0);
	}

	@Test
	public void testSetW() {
		Vector4 p = new Vector4();
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setW(10.0);
		assertEquals(10.0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testSetX() {
		Vector4 p = new Vector4();
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setX(0.5);
		assertEquals(0.5, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
		assertEquals(0, p.getW(), 0);
	}

	@Test
	public void testSetY() {
		Vector4 p = new Vector4();
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setY(1.5);
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(1.5, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);
	}

	@Test
	public void testSetZ() {
		Vector4 p = new Vector4();
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(0, p.getZ(), 0);

		p.setZ(4.5);
		assertEquals(0, p.getW(), 0);
		assertEquals(0, p.getX(), 0);
		assertEquals(0, p.getY(), 0);
		assertEquals(4.5, p.getZ(), 0);
	}

	@Test
	public void testEqualsSameObject() {
		Vector4 p = new Vector4();
		assertTrue(p.equals(p));
	}

	@Test
	public void testEqualsSameValues() {
		double w = 10.0;
		double x = 0.5;
		double y = 1.5;
		double z = 4.5;
		Vector4 v1 = new Vector4(w, x, y, z);
		Vector4 v2 = new Vector4(w, x, y, z);
		assertTrue(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongType() {
		Vector4 p = new Vector4();
		assertFalse(p.equals(new String()));
	}

	@Test
	public void testEqualsWrongW() {
		double x = 0.5;
		double y = 1.5;
		double z = 4.5;
		Vector4 v1 = new Vector4(10.0, x, y, z);
		Vector4 v2 = new Vector4(0, x, y, z);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongX() {
		double w = 10.0;
		double y = 1.5;
		double z = 4.5;
		Vector4 v1 = new Vector4(0.5, y, z, w);
		Vector4 v2 = new Vector4(0, y, z, w);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongY() {
		double w = 10.0;
		double x = 0.5;
		double z = 4.5;
		Vector4 v1 = new Vector4(w, x, 1.5, z);
		Vector4 v2 = new Vector4(w, x, 0, z);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testEqualsWrongZ() {
		double w = 10.0;
		double x = 0.5;
		double y = 1.5;
		Vector4 v1 = new Vector4(w, x, y, 4.5);
		Vector4 v2 = new Vector4(w, x, y, 0);
		assertFalse(v1.equals(v2));
	}

	@Test
	public void testClone() {
		Vector4 v = new Vector4(1, 2, 3, 4);
		assertEquals(v, v.clone());
	}
}
