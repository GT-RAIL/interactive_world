package edu.wpi.rail.jinteractiveworld.model.relevance;

import static org.junit.Assert.*;

import java.util.HashMap;
import java.util.LinkedList;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.Placement;
import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

public class TestHashMapTable {

	@Test
	public void testConstructor() {
		HashMapTable t = new HashMapTable();
		assertNull(t.getReference(new Cup()));
		assertNull(t.getReference(new Cup(), new State()));
		assertNull(t.getReferences(new Cup()));
		assertNotNull(t.getHashMap());
	}

	@Test
	public void testHashMapConstructor() {
		HashMap<Item, LinkedList<Object>> hm = new HashMap<Item, LinkedList<Object>>();
		hm.put(new Cup(), new LinkedList<Object>());
		hm.get(new Cup()).add(new Fork());

		HashMapTable t = new HashMapTable(hm);
		assertEquals(hm, t.getHashMap());
		assertEquals(new Fork(), t.getReference(new Cup()));
		assertNull(t.getReference(new Fork()));
		assertEquals(1, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Cup()).contains(new Fork()));

		hm.get(new Cup()).add(new Plate());

		assertEquals(new Fork(), t.getReference(new Cup()));
		assertNull(t.getReference(new Plate()));
		assertEquals(2, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Cup()).contains(new Fork()));
		assertTrue(t.getReferences(new Cup()).contains(new Plate()));

		hm.put(new Spoon(), new LinkedList<Object>());
		hm.get(new Spoon()).add(new Plate());

		assertEquals(new Plate(), t.getReference(new Spoon()));
		assertNull(t.getReference(new Plate()));
		assertEquals(1, t.getReferences(new Spoon()).size());
		assertEquals(2, t.getReferences(new Cup()).size());
		assertTrue(t.getReferences(new Spoon()).contains(new Plate()));
	}

	@Test
	public void testGetReferenceWithState() {
		HashMap<Item, LinkedList<Object>> hm = new HashMap<Item, LinkedList<Object>>();
		hm.put(new Cup(), new LinkedList<Object>());
		hm.get(new Cup()).add(new Fork());
		hm.get(new Cup()).add(new Plate());

		HashMapTable t = new HashMapTable(hm);
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
		HashMap<Item, LinkedList<Object>> hm = new HashMap<Item, LinkedList<Object>>();
		hm.put(new Cup(), new LinkedList<Object>());
		hm.get(new Cup()).add(new Fork());
		hm.get(new Cup()).add(new Plate());
		hm.put(new Fork(), new LinkedList<Object>());
		hm.get(new Fork()).add(new Plate());

		HashMapTable t = new HashMapTable(hm);
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
