package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;

public class TestRotationMatrix {

	@Test
	public void testConstructor() {
		RotationMatrix.RotationType.values();
		RotationMatrix.RotationType.valueOf("X_ROTATION");
		RotationMatrix r = new RotationMatrix();
		assertEquals(1, r.getR11(), 0);
		assertEquals(0, r.getR12(), 0);
		assertEquals(0, r.getR13(), 0);
		assertEquals(0, r.getR21(), 0);
		assertEquals(1, r.getR22(), 0);
		assertEquals(0, r.getR23(), 0);
		assertEquals(0, r.getR31(), 0);
		assertEquals(0, r.getR32(), 0);
		assertEquals(1, r.getR33(), 0);

		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.X_ROTATION));
		assertEquals(-0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Y_ROTATION));
		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION));
	}

	@Test
	public void testThetaTypeConstructorXRotation() {
		double theta = Math.PI / 2.0;

		RotationMatrix r = new RotationMatrix(theta,
				RotationMatrix.RotationType.X_ROTATION);

		assertEquals(1, r.getR11(), 0);
		assertEquals(0, r.getR12(), 0);
		assertEquals(0, r.getR13(), 0);
		assertEquals(0, r.getR21(), 0);
		assertEquals(Math.cos(theta), r.getR22(), 0);
		assertEquals(-Math.sin(theta), r.getR23(), 0);
		assertEquals(0, r.getR31(), 0);
		assertEquals(Math.sin(theta), r.getR32(), 0);
		assertEquals(Math.cos(theta), r.getR33(), 0);

		assertEquals(theta,
				r.getRotationAboutAxis(RotationMatrix.RotationType.X_ROTATION));
		assertEquals(-0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Y_ROTATION));
		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION));
	}

	@Test
	public void testThetaTypeConstructorYRotation() {
		double theta = Math.PI / 2.0;

		RotationMatrix r = new RotationMatrix(theta,
				RotationMatrix.RotationType.Y_ROTATION);

		assertEquals(Math.cos(theta), r.getR11(), 0);
		assertEquals(0, r.getR12(), 0);
		assertEquals(Math.sin(theta), r.getR13(), 0);
		assertEquals(0, r.getR21(), 0);
		assertEquals(1, r.getR22(), 0);
		assertEquals(0, r.getR23(), 0);
		assertEquals(-Math.sin(theta), r.getR31(), 0);
		assertEquals(0, r.getR32(), 0);
		assertEquals(Math.cos(theta), r.getR33(), 0);

		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.X_ROTATION));
		assertEquals(theta,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Y_ROTATION));
		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION));
	}

	@Test
	public void testThetaTypeConstructorZRotation() {
		double theta = Math.PI / 2.0;

		RotationMatrix r = new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION);

		assertEquals(Math.cos(theta), r.getR11(), 0);
		assertEquals(-Math.sin(theta), r.getR12(), 0);
		assertEquals(0, r.getR13(), 0);
		assertEquals(Math.sin(theta), r.getR21(), 0);
		assertEquals(Math.cos(theta), r.getR22(), 0);
		assertEquals(0, r.getR23(), 0);
		assertEquals(0, r.getR31(), 0);
		assertEquals(0, r.getR32(), 0);
		assertEquals(1, r.getR33(), 0);

		assertEquals(0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.X_ROTATION));
		assertEquals(-0.0,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Y_ROTATION));
		assertEquals(theta,
				r.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION));
	}

	@Test
	public void testFullConstructor() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);

		assertEquals(1, r.getR11(), 0);
		assertEquals(2, r.getR12(), 0);
		assertEquals(3, r.getR13(), 0);
		assertEquals(4, r.getR21(), 0);
		assertEquals(5, r.getR22(), 0);
		assertEquals(6, r.getR23(), 0);
		assertEquals(7, r.getR31(), 0);
		assertEquals(8, r.getR32(), 0);
		assertEquals(9, r.getR33(), 0);
	}

	@Test
	public void testSetRotation() {
		RotationMatrix r = new RotationMatrix();
		double theta = Math.PI / 2.0;
		r.setRotation(theta, RotationMatrix.RotationType.X_ROTATION);

		assertEquals(1, r.getR11(), 0);
		assertEquals(0, r.getR12(), 0);
		assertEquals(0, r.getR13(), 0);
		assertEquals(0, r.getR21(), 0);
		assertEquals(Math.cos(theta), r.getR22(), 0);
		assertEquals(-Math.sin(theta), r.getR23(), 0);
		assertEquals(0, r.getR31(), 0);
		assertEquals(Math.sin(theta), r.getR32(), 0);
		assertEquals(Math.cos(theta), r.getR33(), 0);
	}

	@Test
	public void testSetR11() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR11(10);
		assertEquals(10, r.getR11(), 0);
	}

	@Test
	public void testSetR12() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR12(10);
		assertEquals(10, r.getR12(), 0);
	}

	@Test
	public void testSetR13() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR13(10);
		assertEquals(10, r.getR13(), 0);
	}

	@Test
	public void testSetR21() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR21(10);
		assertEquals(10, r.getR21(), 0);
	}

	@Test
	public void testSetR22() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR22(10);
		assertEquals(10, r.getR22(), 0);
	}

	@Test
	public void testSetR23() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR23(10);
		assertEquals(10, r.getR23(), 0);
	}

	@Test
	public void testSetR31() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR31(10);
		assertEquals(10, r.getR31(), 0);
	}

	@Test
	public void testSetR32() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR32(10);
		assertEquals(10, r.getR32(), 0);
	}

	@Test
	public void testSetR33() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		r.setR33(10);
		assertEquals(10, r.getR33(), 0);
	}
	
	@Test
	public void testToVector4() {
		RotationMatrix r = new RotationMatrix(Math.PI / 4.0, RotationMatrix.RotationType.Z_ROTATION);
		Vector4 v4 = r.toVector4();
		assertEquals(0.9238795325112867, v4.getW(), 0);
		assertEquals(0, v4.getX(), 0);
		assertEquals(0, v4.getY(), 0);
		assertEquals(0.3826834323650897, v4.getZ(), 0.0000000000000001);
	}

	@Test
	public void testEqualsSameObject() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		assertTrue(r.equals(r));
	}

	@Test
	public void testEqualsSameValues() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		assertTrue(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongType() {
		RotationMatrix r = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		assertFalse(r.equals(new String()));
	}

	@Test
	public void testEqualsWrongR11() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(0, 2, 3, 4, 5, 6, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR12() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 0, 3, 4, 5, 6, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR13() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 0, 4, 5, 6, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR21() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 0, 5, 6, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR22() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 0, 6, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR23() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 5, 0, 7, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR31() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 5, 6, 0, 8, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR32() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 0, 9);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testEqualsWrongR33() {
		RotationMatrix r1 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
		RotationMatrix r2 = new RotationMatrix(1, 2, 3, 4, 5, 6, 7, 8, 0);
		assertFalse(r1.equals(r2));
	}

	@Test
	public void testClone() {
		RotationMatrix r = new RotationMatrix(Math.PI / 2.0,
				RotationMatrix.RotationType.Z_ROTATION);
		assertEquals(r, r.clone());
	}
}
