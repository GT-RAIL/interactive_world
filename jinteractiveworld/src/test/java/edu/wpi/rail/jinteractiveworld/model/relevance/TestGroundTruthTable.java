package edu.wpi.rail.jinteractiveworld.model.relevance;

import static org.junit.Assert.*;

import java.util.LinkedList;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.Placement;
import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.relevance.GroundTruthTable;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

public class TestGroundTruthTable {

	@Test
	public void testConstructor() {
		GroundTruthTable t = new GroundTruthTable();
		assertNull(t.getReference(new Cup()));
		assertNull(t.getReference(new Cup(), new State()));
		assertNull(t.getReferences(new Cup()));
		assertNotNull(t.getHashMap());
	}

	@Test
	public void testAddReference() {
		GroundTruthTable t = new GroundTruthTable();
		t.addReference(new Fork(), new Cup());
		assertEquals(new Fork(), t.getReference(new Cup()));
		assertNull(t.getReference(new Fork()));
		assertEquals(1, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Cup()).contains(new Fork()));

		t.addReference(new Plate(), new Cup());
		assertEquals(new Fork(), t.getReference(new Cup()));
		assertNull(t.getReference(new Plate()));
		assertEquals(2, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Cup()).contains(new Fork()));
		assertTrue(t.getReferences(new Cup()).contains(new Plate()));

		t.addReference(new Plate(), new Spoon());
		assertEquals(new Plate(), t.getReference(new Spoon()));
		assertNull(t.getReference(new Plate()));
		assertEquals(1, t.getReferences(new Spoon()).size());
		assertEquals(2, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Spoon()).contains(new Plate()));
	}

	@Test
	public void testGetReferenceWithState() {
		GroundTruthTable t = new GroundTruthTable();
		t.addReference(new Fork(), new Cup());
		t.addReference(new Plate(), new Cup());
		assertEquals(2, t.getReferences(new Cup()).size());

		State s = new State();
		s.add(new Placement(new Cup(), new Transform()));
		assertNull(t.getReference(new Cup(), s));
		s.add(new Placement(new Plate(), new Transform()));
		assertEquals(new Plate(), t.getReference(new Cup(), s));
		s.add(new Placement(new Spoon(), new Transform()));
		assertEquals(new Plate(), t.getReference(new Cup(), s));
		s.add(new Placement(new Fork(), new Transform()));
		assertEquals(new Fork(), t.getReference(new Cup(), s));
	}

	@Test
	public void testGetReferencesWithState() {
		GroundTruthTable t = new GroundTruthTable();
		t.addReference(new Fork(), new Cup());
		t.addReference(new Plate(), new Cup());
		t.addReference(new Plate(), new Fork());
		assertEquals(2, t.getReferences(new Cup()).size());
		assertEquals(1, t.getReferences(new Fork()).size());

		State s = new State();
		s.add(new Placement(new Cup(), new Transform()));
		assertNull(t.getReferences(new Spoon(), s));
		assertNull(t.getReferences(new Cup(), s));
		s.add(new Placement(new Plate(), new Transform()));
		LinkedList<Object> refs = t.getReferences(new Cup(), s);
		assertEquals(1, refs.size());
		assertEquals(new Plate(), refs.get(0));
	}
}
