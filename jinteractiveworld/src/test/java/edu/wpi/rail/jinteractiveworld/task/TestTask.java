package edu.wpi.rail.jinteractiveworld.task;

import static org.junit.Assert.*;

import java.util.HashSet;
import java.util.Set;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.ClusteringModel;
import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.model.relevance.GroundTruthTable;
import edu.wpi.rail.jinteractiveworld.model.relevance.RelevanceTable;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

public class TestTask {

	@Test
	public void testNameTableAndModelsConstructor() {
		String name = "test";
		RelevanceTable table = new GroundTruthTable();
		HashSet<Model> models = new HashSet<Model>();
		models.add(new ClusteringModel(new DataSet(), new Fork(), new Spoon()));
		Task t = new DummyTask(name, table, models);
		t.execute();
		assertEquals(name, t.getName());
		assertEquals(table, t.getRelevanceTable());
		assertEquals(models, t.getModels());
		assertEquals(0, t.getAssociatedItems().size());

		assertEquals(models.size(), t.getModels().size());
		for (Model m : models) {
			assertTrue(t.getModels().contains(m));
		}
	}

	@Test
	public void testNameTableAndItemsConstructor() {
		String name = "test";
		RelevanceTable table = new GroundTruthTable();
		HashSet<Model> models = new HashSet<Model>();
		models.add(new ClusteringModel(new DataSet(), new Fork(), new Spoon()));
		HashSet<Item> items = new HashSet<Item>();
		items.add(new Cup());
		items.add(new Spoon());

		Task t = new DummyTask(name, table, models, items);
		t.execute();
		assertEquals(name, t.getName());
		assertEquals(table, t.getRelevanceTable());
		assertEquals(items.size(), t.getAssociatedItems().size());

		for (Item i : items) {
			assertTrue(t.getAssociatedItems().contains(i));
		}

		assertEquals(models.size(), t.getModels().size());
		for (Model m : models) {
			assertTrue(t.getModels().contains(m));
		}
	}

	@Test
	public void testSetName() {
		String name1 = "test1";
		Task t = new DummyTask(name1, new GroundTruthTable(),
				new HashSet<Model>());
		assertEquals(name1, t.getName());
		String name2 = "test2";
		t.setName(name2);
		assertEquals(name2, t.getName());
	}

	@Test
	public void testSetRelevanceTable() {
		RelevanceTable table1 = new GroundTruthTable();
		Task t = new DummyTask("test", table1, new HashSet<Model>());
		assertEquals(table1, t.getRelevanceTable());
		RelevanceTable table2 = new GroundTruthTable();
		t.setRelevanceTable(table2);
		assertEquals(table2, t.getRelevanceTable());
	}

	@Test
	public void testAddItem() {
		Task t = new DummyTask("test", new GroundTruthTable(),
				new HashSet<Model>());
		assertEquals(0, t.getAssociatedItems().size());
		Cup c = new Cup();
		t.addAssociatedItem(c);
		assertEquals(1, t.getAssociatedItems().size());
		assertTrue(t.getAssociatedItems().contains(c));
		Spoon s = new Spoon();
		t.addAssociatedItem(s);
		assertEquals(2, t.getAssociatedItems().size());
		assertTrue(t.getAssociatedItems().contains(s));
	}

	@Test
	public void testAddItems() {
		Task t = new DummyTask("test", new GroundTruthTable(),
				new HashSet<Model>());
		assertEquals(0, t.getAssociatedItems().size());
		HashSet<Item> items = new HashSet<Item>();
		items.add(new Cup());
		items.add(new Spoon());

		t.addAssociatedItems(items);
		assertEquals(items.size(), t.getAssociatedItems().size());
		for (Item i : items) {
			assertTrue(t.getAssociatedItems().contains(i));
		}
	}

	private class DummyTask extends Task {

		public DummyTask(String name, RelevanceTable table, Set<Model> models) {
			super(name, table, models);
		}

		public DummyTask(String name, RelevanceTable table, Set<Model> models,
				Set<Item> items) {
			super(name, table, models, items);
		}

		public void execute() {
		}
	}
}
