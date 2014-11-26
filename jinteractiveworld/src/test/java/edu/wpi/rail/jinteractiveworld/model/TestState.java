package edu.wpi.rail.jinteractiveworld.model;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Object;

public class TestState {

	@Test
	public void testConstructor() {
		State s = new State();
		assertEquals(0, s.getPlacements().size());
		assertEquals(0, s.getObjects().size());
		assertEquals(0, s.getTransforms().size());
	}

	@Test
	public void testAddWithPlacement() {
		State s = new State();
		assertEquals(0, s.getPlacements().size());

		Placement p1 = new Placement(new Cup(), new Transform());
		s.add(p1);
		assertEquals(1, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(p1));
		Placement p2 = new Placement(new Fork(), new Transform(
				new RotationMatrix(), new Vector3(0.5)));
		s.add(p2);
		assertEquals(2, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(p2));
	}

	@Test
	public void testAddWithObjectAndTransform() {
		State s = new State();
		assertEquals(0, s.getPlacements().size());

		Object o1 = new Cup();
		Transform tf1 = new Transform();
		s.add(o1, tf1);
		assertEquals(1, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(new Placement(o1, tf1)));

		Object o2 = new Fork();
		Transform tf2 = new Transform(new RotationMatrix(), new Vector3(0.5));
		s.add(o2, tf2);
		assertEquals(2, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(new Placement(o2, tf2)));
	}

	@Test
	public void testAddWithObjectRotationMatrixAndVector3() {
		State s = new State();
		assertEquals(0, s.getPlacements().size());

		Object o1 = new Cup();
		RotationMatrix r1 = new RotationMatrix();
		Vector3 v1 = new Vector3();
		s.add(o1, r1, v1);
		assertEquals(1, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(new Placement(o1, r1, v1)));

		Object o2 = new Fork();
		RotationMatrix r2 = new RotationMatrix(Math.PI,
				RotationMatrix.RotationType.Z_ROTATION);
		Vector3 v2 = new Vector3(0.5);
		s.add(o2, r2, v2);
		assertEquals(2, s.getPlacements().size());
		assertTrue(s.getPlacements().contains(new Placement(o2, r2, v2)));
	}

	@Test
	public void testGetObjects() {
		State s = new State();
		assertEquals(0, s.getObjects().size());

		Placement p1 = new Placement(new Cup(), new Transform());
		s.add(p1);
		assertEquals(1, s.getObjects().size());
		assertTrue(s.getObjects().contains(p1.getObject()));
		Placement p2 = new Placement(new Fork(), new Transform(
				new RotationMatrix(), new Vector3(0.5)));
		s.add(p2);
		assertEquals(2, s.getObjects().size());
		assertTrue(s.getObjects().contains(p2.getObject()));
	}

	@Test
	public void testGetPoints() {
		State s = new State();
		assertEquals(0, s.getTransforms().size());

		Placement p1 = new Placement(new Cup(), new Transform());
		s.add(p1);
		assertEquals(1, s.getTransforms().size());
		assertTrue(s.getTransforms().contains(p1.getTransform()));
		Placement p2 = new Placement(new Fork(), new Transform(
				new RotationMatrix(), new Vector3(0.5)));
		s.add(p2);
		assertEquals(2, s.getTransforms().size());
		assertTrue(s.getTransforms().contains(p2.getTransform()));
	}
}
