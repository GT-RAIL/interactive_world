package edu.wpi.rail.jinteractiveworld.model;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

public class TestDataSetModel {

	@Test
	public void testConstructor() {
		DataSet ds = new DataSet();
		ds.add(1, 2, 3, 4);
		ds.add(5, 6, 7, 8);

		DataSetModel model = new DummyDataSetModel(ds, new Chair(), new Fork());
		assertEquals(2, model.size());
		assertEquals(new Chair(), model.getReference());
		assertEquals(new Fork(), model.getTarget());
		assertNull(model.getHighestPlacementLocation());
		model.train();
		assertEquals(0.0, model.getDecisionValue());
	}

	@Test
	public void testAdd() {
		DataSetModel model = new DummyDataSetModel(new DataSet(), new Chair(),
				new Fork());
		assertEquals(0, model.size());

		model.add(new Transform());
		assertEquals(1, model.size());
		assertEquals(1, model.getData().size());
		assertEquals(new Transform(), model.getData().get(0));
		model.add(new Transform(new RotationMatrix(), new Vector3(0.5)));
		assertEquals(2, model.size());
		assertEquals(2, model.getData().size());
		assertEquals(new Transform(new RotationMatrix(), new Vector3(0.5)),
				model.getData().get(1));
	}

	private class DummyDataSetModel extends DataSetModel {

		public DummyDataSetModel(DataSet data, Object reference, Item target) {
			super(data, reference, target);
		}

		@Override
		public Transform getHighestPlacementLocation() {
			return null;
		}

		@Override
		public void train() {
		}

		public double getDecisionValue() {
			return 0;
		}
	}
}
