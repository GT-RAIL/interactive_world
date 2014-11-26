package edu.wpi.rail.jinteractiveworld.model;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.Cup;

public class TestClusteringModel {
	
	@Test
	public void testInvalidDataSet() {
		ClusteringModel model = new ClusteringModel(null, null, null);
		assertNull(model.getHighestPlacementLocation());
	}

	@Test
	public void testGetHighestPlacementLocation() {		
		DataSet data = new DataSet();

		data.add(1, 1, 0, Math.PI);
		data.add(0.5, 0.5, 0, Math.PI / 2.0);
		data.add(0.5, 1.5, 0, Math.PI);
		data.add(1.5, 0.5, 0, Math.PI / 2.0);
		data.add(1.5, 1.5, 0, Math.PI);
		
		data.add(-5, -5, 0, Math.PI);
		data.add(-4.875, -4.875, 0, Math.PI / 2.0);
		data.add(-4.875, -5.125, 0, Math.PI);
		data.add(-5.125, -4.875, 0, Math.PI / 2.0);
		data.add(-5.125, -5.125, 0, Math.PI);

		ClusteringModel model = new ClusteringModel(data, new Chair(),
				new Cup());

		Transform tf = model.getHighestPlacementLocation();
		assertEquals(-5.0, tf.getVector3().getX());
		assertEquals(-5.0, tf.getVector3().getY());
		assertEquals(0.0, tf.getVector3().getZ());
		assertEquals(
				0.0,
				tf.getRotationMatrix().getRotationAboutAxis(
						RotationMatrix.RotationType.X_ROTATION));
		assertEquals(
				-0.0,
				tf.getRotationMatrix().getRotationAboutAxis(
						RotationMatrix.RotationType.Y_ROTATION));
		assertEquals((Math.PI * 4.0) / 5.0, tf.getRotationMatrix()
				.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION));
		assertEquals(2.083259, model.getDecisionValue(), 0.000001);
	}
}
