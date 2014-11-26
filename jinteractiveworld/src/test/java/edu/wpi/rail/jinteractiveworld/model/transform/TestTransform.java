package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;

public class TestTransform {

	@Test
	public void testConstructor() {
		Transform tf = new Transform();
		assertEquals(new RotationMatrix(), tf.getRotationMatrix());
		assertEquals(new Vector3(), tf.getVector3());
	}

	@Test
	public void testRotationMatrixAndVector3Constructor() {
		double theta = Math.PI / 2.0;
		double x = 0.5;
		double y = 1.5;
		double z = 2.5;

		Transform tf = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		assertEquals(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), tf.getRotationMatrix());
		assertEquals(new Vector3(x, y, z), tf.getVector3());
	}

	@Test
	public void testFullConstructor() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);

		assertEquals(1, t.getR11(), 0);
		assertEquals(2, t.getR12(), 0);
		assertEquals(3, t.getR13(), 0);
		assertEquals(4, t.getR14(), 0);
		assertEquals(5, t.getR21(), 0);
		assertEquals(6, t.getR22(), 0);
		assertEquals(7, t.getR23(), 0);
		assertEquals(8, t.getR24(), 0);
		assertEquals(9, t.getR31(), 0);
		assertEquals(10, t.getR32(), 0);
		assertEquals(11, t.getR33(), 0);
		assertEquals(12, t.getR34(), 0);
		assertEquals(13, t.getR41(), 0);
		assertEquals(14, t.getR42(), 0);
		assertEquals(15, t.getR43(), 0);
		assertEquals(16, t.getR44(), 0);
	}

	@Test
	public void testSetRotation() {
		double theta = Math.PI / 2.0;
		double x = 0.5;
		double y = 1.5;
		double z = 2.5;

		Transform tf = new Transform(new RotationMatrix(theta / 2.0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		tf.setRotation(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION));
		assertEquals(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), tf.getRotationMatrix());
		assertEquals(new Vector3(x, y, z), tf.getVector3());
	}

	@Test
	public void testSetTranslation() {
		double theta = Math.PI / 2.0;
		double x = 0.5;
		double y = 1.5;
		double z = 2.5;

		Transform tf = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3());
		tf.setTranslation(new Vector3(x, y, z));
		assertEquals(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), tf.getRotationMatrix());
		assertEquals(new Vector3(x, y, z), tf.getVector3());
	}

	@Test
	public void testSetR11() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR11(100);
		assertEquals(100, t.getR11(), 0);
	}

	@Test
	public void testSetR12() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR12(100);
		assertEquals(100, t.getR12(), 0);
	}

	@Test
	public void testSetR13() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR13(100);
		assertEquals(100, t.getR13(), 0);
	}

	@Test
	public void testSetR14() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR14(100);
		assertEquals(100, t.getR14(), 0);
	}

	@Test
	public void testSetR21() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR21(100);
		assertEquals(100, t.getR21(), 0);
	}

	@Test
	public void testSetR22() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR22(100);
		assertEquals(100, t.getR22(), 0);
	}

	@Test
	public void testSetR23() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR23(100);
		assertEquals(100, t.getR23(), 0);
	}

	@Test
	public void testSetR24() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR24(100);
		assertEquals(100, t.getR24(), 0);
	}

	@Test
	public void testSetR31() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR31(100);
		assertEquals(100, t.getR31(), 0);
	}

	@Test
	public void testSetR32() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR32(100);
		assertEquals(100, t.getR32(), 0);
	}

	@Test
	public void testSetR33() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR33(100);
		assertEquals(100, t.getR33(), 0);
	}

	@Test
	public void testSetR34() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR34(100);
		assertEquals(100, t.getR34(), 0);
	}

	@Test
	public void testSetR41() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR41(100);
		assertEquals(100, t.getR41(), 0);
	}

	@Test
	public void testSetR42() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR42(100);
		assertEquals(100, t.getR42(), 0);
	}

	@Test
	public void testSetR43() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR43(100);
		assertEquals(100, t.getR43(), 0);
	}

	@Test
	public void testSetR44() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		t.setR44(100);
		assertEquals(100, t.getR44(), 0);
	}

	@Test
	public void testGetInverse() {
		Transform tf1 = new Transform(4, 0, 0, 0, 0, 0, 2, 0, 0, 1, 2, 0, 1, 0,
				0, 1);
		Transform tf2 = tf1.getInverse();
		assertEquals(0.25, tf2.getR11(), 0);
		assertEquals(0, tf2.getR12(), 0);
		assertEquals(0, tf2.getR13(), 0);
		assertEquals(0, tf2.getR14(), 0);
		assertEquals(0, tf2.getR21(), 0);
		assertEquals(-1, tf2.getR22(), 0);
		assertEquals(1, tf2.getR23(), 0);
		assertEquals(0, tf2.getR24(), 0);
		assertEquals(0, tf2.getR31(), 0);
		assertEquals(0.5, tf2.getR32(), 0);
		assertEquals(0, tf2.getR33(), 0);
		assertEquals(0, tf2.getR34(), 0);
		assertEquals(-0.25, tf2.getR41(), 0);
		assertEquals(0, tf2.getR42(), 0);
		assertEquals(0, tf2.getR43(), 0);
		assertEquals(1, tf2.getR44(), 0);
	}

	@Test
	public void testGetInverseInvalid() {
		Transform t = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 16);
		assertNull(t.getInverse());
	}

	@Test
	public void testMultiply() {
		Transform tf1 = new Transform(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
				13, 14, 15, 16);
		Transform tf2 = new Transform(16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6,
				5, 4, 3, 2, 1);
		assertEquals(new Transform(80, 70, 60, 50, 240, 214, 188, 162, 400,
				358, 316, 274, 560, 502, 444, 386), tf1.multiply(tf2));
	}

	@Test
	public void testEqualsSameObject() {
		Transform tf = new Transform(new RotationMatrix(0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3());
		assertTrue(tf.equals(tf));
	}

	@Test
	public void testEqualsSameValues() {
		double theta = Math.PI / 2.0;
		double x = 0.5;
		double y = 1.5;
		double z = 2.5;

		Transform tf1 = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		Transform tf2 = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		assertTrue(tf1.equals(tf2));
	}

	@Test
	public void testEqualsWrongType() {
		Transform tf = new Transform(new RotationMatrix(0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3());
		assertFalse(tf.equals(new String()));
	}

	@Test
	public void testEqualsWrongRotationMatrix() {
		double x = 0.5;
		double y = 1.5;
		double z = 2.5;

		Transform tf1 = new Transform(new RotationMatrix(0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		Transform tf2 = new Transform(new RotationMatrix(1,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		assertFalse(tf1.equals(tf2));
	}

	@Test
	public void testEqualsWrongVector() {
		double theta = Math.PI / 2.0;

		Transform tf1 = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(0));
		Transform tf2 = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(1));
		assertFalse(tf1.equals(tf2));
	}

	@Test
	public void testClone() {
		Transform tf = new Transform(new RotationMatrix(0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(1, 2, 3));
		assertEquals(tf, tf.clone());
	}
}
