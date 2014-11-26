package edu.wpi.rail.jinteractiveworld.model.relevance;

import static org.junit.Assert.*;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.relevance.ModelSetTable;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.DiningTable;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

public class TestModelSetTable {

	@Test
	public void testDummyModel() {
		Object ref = new Spoon();
		Item target = new Fork();
		double value = 0.5;

		DummyModel model = new DummyModel(ref, target, value);
		model.train();
		model.add(null);
		assertEquals(0, model.size());
		assertNull(model.getData());
		assertNull(model.getHighestPlacementLocation());
		assertEquals(ref, model.getReference());
		assertEquals(target, model.getTarget());
		assertEquals(value, model.getDecisionValue());
	}

	@Test
	public void testConstructor() {
		HashSet<Model> models = new HashSet<Model>();
		ModelSetTable table = new ModelSetTable(models);
		assertNull(table.getReference(new Spoon()));
		assertNull(table.getReference(new Spoon(), new State()));
		assertNull(table.getReferences(new Spoon()));
		assertNull(table.getReferences(new Spoon(), new State()));
	}

	@Test
	public void testGetReference() {
		HashSet<Model> models = new HashSet<Model>();
		models.add(new DummyModel(new Chair(), new Cup(), 0.2));
		models.add(new DummyModel(new Spoon(), new Cup(), 0.1));
		models.add(new DummyModel(new Plate(), new Cup(), 0.3));
		models.add(new DummyModel(new DiningTable(), new Cup(), 0.01));
		models.add(new DummyModel(new Fork(), new Cup(), 1.3));
		models.add(new DummyModel(new Chair(), new Spoon(), 1.01));
		models.add(new DummyModel(new Fork(), new Spoon(), 1.63));

		ModelSetTable table = new ModelSetTable(models);
		assertEquals(new DiningTable(), table.getReference(new Cup()));
		assertEquals(new Chair(), table.getReference(new Spoon()));
		assertNull(table.getReference(new Fork()));
	}

	@Test
	public void testGetReferences() {
		HashSet<Model> models = new HashSet<Model>();
		models.add(new DummyModel(new Chair(), new Cup(), 0.2));
		models.add(new DummyModel(new Spoon(), new Cup(), 0.1));
		models.add(new DummyModel(new Plate(), new Cup(), 0.3));
		models.add(new DummyModel(new DiningTable(), new Cup(), 0.01));
		models.add(new DummyModel(new Fork(), new Cup(), 1.3));
		models.add(new DummyModel(new Chair(), new Spoon(), 1.01));
		models.add(new DummyModel(new Fork(), new Spoon(), 1.63));

		ModelSetTable table = new ModelSetTable(models);
		LinkedList<Object> list1 = table.getReferences(new Cup());
		assertEquals(5, list1.size());
		assertEquals(new DiningTable(), list1.get(0));
		assertEquals(new Spoon(), list1.get(1));
		assertEquals(new Chair(), list1.get(2));
		assertEquals(new Plate(), list1.get(3));
		assertEquals(new Fork(), list1.get(4));

		LinkedList<Object> list2 = table.getReferences(new Spoon());
		assertEquals(2, list2.size());
		assertEquals(new Chair(), list2.get(0));
		assertEquals(new Fork(), list2.get(1));

		assertNull(table.getReferences(new Fork()));
	}

	@Test
	public void testGetReferenceWithState() {
		HashSet<Model> models = new HashSet<Model>();
		models.add(new DummyModel(new Chair(), new Cup(), 0.2));
		models.add(new DummyModel(new Spoon(), new Cup(), 0.1));
		models.add(new DummyModel(new Plate(), new Cup(), 0.3));
		models.add(new DummyModel(new DiningTable(), new Cup(), 0.01));
		models.add(new DummyModel(new Fork(), new Cup(), 1.3));
		models.add(new DummyModel(new Chair(), new Spoon(), 1.01));
		models.add(new DummyModel(new Fork(), new Spoon(), 1.63));

		State s = new State();
		s.add(new Chair(), new Transform());
		s.add(new Plate(), new Transform());
		s.add(new Fork(), new Transform());
		
		ModelSetTable table = new ModelSetTable(models);
		assertEquals(new Chair(), table.getReference(new Cup(), s));
		assertEquals(new Chair(), table.getReference(new Spoon(), s));
		assertNull(table.getReference(new Fork()));
	}

	@Test
	public void testGetReferencesWithState() {
		HashSet<Model> models = new HashSet<Model>();
		models.add(new DummyModel(new Chair(), new Cup(), 0.2));
		models.add(new DummyModel(new Spoon(), new Cup(), 0.1));
		models.add(new DummyModel(new Plate(), new Cup(), 0.3));
		models.add(new DummyModel(new DiningTable(), new Cup(), 0.01));
		models.add(new DummyModel(new Fork(), new Cup(), 1.3));
		models.add(new DummyModel(new Chair(), new Spoon(), 1.01));
		models.add(new DummyModel(new Fork(), new Spoon(), 1.63));

		State s = new State();
		s.add(new Chair(), new Transform());
		s.add(new Plate(), new Transform());

		ModelSetTable table = new ModelSetTable(models);
		LinkedList<Object> list1 = table.getReferences(new Cup(), s);
		assertEquals(2, list1.size());
		assertEquals(new Chair(), list1.get(0));
		assertEquals(new Plate(), list1.get(1));

		LinkedList<Object> list2 = table.getReferences(new Spoon(), s);
		assertEquals(1, list2.size());
		assertEquals(new Chair(), list2.get(0));

		assertNull(table.getReferences(new Fork()));
	}

	private class DummyModel implements Model {

		private Object reference;
		private Item target;
		private double value;

		public DummyModel(Object reference, Item target, double value) {
			this.reference = reference;
			this.target = target;
			this.value = value;
		}

		@Override
		public void add(Transform tf) {
		}

		@Override
		public int size() {
			return 0;
		}

		@Override
		public List<Transform> getData() {
			return null;
		}

		@Override
		public Object getReference() {
			return this.reference;
		}

		@Override
		public Item getTarget() {
			return this.target;
		}

		@Override
		public void train() {
		}

		@Override
		public Transform getHighestPlacementLocation() {
			return null;
		}

		@Override
		public double getDecisionValue() {
			return this.value;
		}

	}
}
