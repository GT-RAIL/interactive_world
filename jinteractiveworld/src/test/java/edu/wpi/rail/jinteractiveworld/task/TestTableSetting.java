package edu.wpi.rail.jinteractiveworld.task;

import static org.junit.Assert.*;

import java.lang.reflect.Field;
import java.util.HashSet;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.ClusteringModel;
import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.model.relevance.GroundTruthTable;
import edu.wpi.rail.jinteractiveworld.model.relevance.RelevanceTable;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

public class TestTableSetting {

	@Test
	public void testConstructor() throws SecurityException,
			NoSuchFieldException, IllegalArgumentException,
			IllegalAccessException {
		Field f1 = TableSetting.class.getDeclaredField("NAME");
		f1.setAccessible(true);
		Field f2 = TableSetting.class.getDeclaredField("ITEMS");
		f2.setAccessible(true);

		RelevanceTable table = new GroundTruthTable();
		HashSet<Model> models = new HashSet<Model>();
		models.add(new ClusteringModel(new DataSet(), new Fork(), new Spoon()));
		TableSetting ts = new TableSetting(table, models);
		assertEquals((String) f1.get(ts), ts.getName());
		assertEquals(table, ts.getRelevanceTable());
		assertEquals(models, ts.getModels());

		Item[] items = (Item[]) f2.get(ts);
		for (Item i : items) {
			assertTrue(ts.getAssociatedItems().contains(i));
		}

		assertEquals(models.size(), ts.getModels().size());
		for (Model m : models) {
			assertTrue(ts.getModels().contains(m));
		}
	}

	@Test
	public void testMain() {
		TableSetting.main(null);
	}
}
