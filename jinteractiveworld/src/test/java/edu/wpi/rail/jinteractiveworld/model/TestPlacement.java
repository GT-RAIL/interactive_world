package edu.wpi.rail.jinteractiveworld.model;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.Placement;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Object;

public class TestPlacement {

	@Test
	public void testRotationMatrixAndVector3Constructor() {
		Object o = new Cup();
		RotationMatrix r = new RotationMatrix();
		Vector3 v = new Vector3();

		Placement placement = new Placement(o, r, v);
		assertEquals(o, placement.getObject());
		assertEquals(new Transform(r, v), placement.getTransform());
	}

	@Test
	public void testTransformConstructor() {
		Object o = new Cup();
		Transform tf = new Transform();

		Placement placement = new Placement(o, tf);
		assertEquals(o, placement.getObject());
		assertEquals(tf, placement.getTransform());
	}

	@Test
	public void testSetTransform() {
		Transform tf1 = new Transform();
		Placement placement = new Placement(new Cup(), tf1);
		assertEquals(tf1, placement.getTransform());
		Transform tf2 = new Transform(new RotationMatrix(),
				new Vector3(1, 2, 3));
		placement.setTransform(tf2);
		assertEquals(tf2, placement.getTransform());
	}

	@Test
	public void testSetObject() {
		Object o1 = new Cup();
		Placement placement = new Placement(o1, new Transform());
		assertEquals(o1, placement.getObject());
		Object o2 = new Fork();
		placement.setObject(o2);
		assertEquals(o2, placement.getObject());
	}

	@Test
	public void testEqualsSameObject() {
		Placement placement = new Placement(new Cup(), new Transform());
		assertTrue(placement.equals(placement));
	}

	@Test
	public void testEqualsSameValues() {
		Object o1 = new Cup();
		Transform tf1 = new Transform();
		Object o2 = new Cup();
		Transform tf2 = new Transform();

		Placement placement1 = new Placement(o1, tf1);
		Placement placement2 = new Placement(o2, tf2);
		assertTrue(placement1.equals(placement2));
	}

	@Test
	public void testEqualsWrongType() {
		Placement placement = new Placement(new Cup(), new Transform());
		assertFalse(placement.equals(new String()));
	}

	@Test
	public void testEqualsWrongObject() {
		Placement placement1 = new Placement(new Cup(), new Transform());
		Placement placement2 = new Placement(new Fork(), new Transform());
		assertFalse(placement1.equals(placement2));
	}

	@Test
	public void testEqualsWrongTransform() {
		Object o = new Cup();
		Placement placement1 = new Placement(o, new Transform());
		Placement placement2 = new Placement(o, new Transform(
				new RotationMatrix(), new Vector3(1, 2, 3)));
		assertFalse(placement1.equals(placement2));
	}
}
