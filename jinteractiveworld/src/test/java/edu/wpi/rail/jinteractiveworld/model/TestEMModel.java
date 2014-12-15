package edu.wpi.rail.jinteractiveworld.model;

import static org.junit.Assert.*;

import edu.wpi.rail.jinteractiveworld.data.DataPoint;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;
import edu.wpi.rail.jinteractiveworld.data.DataSet;
import org.junit.Test;

public class TestEMModel {
	
	@Test
	public void testInvalidDataSet() {
		EMModel model = new EMModel(null, null);
		assertNull(model.getPlacementLocation());
	}

	@Test
	public void testGetHighestPlacementLocation() {		
		DataSet data = new DataSet(new Item(), new Room(), new Surface(), "test");

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

		EMModel model = new EMModel(data, EMModel.RankingFunction.CUSTOM);

		Placement p = model.getPlacementLocation();
		assertEquals("test", p.getReferenceFrameId());
		assertEquals(model.getItem(), p.getItem());
		assertEquals(model.getRoom(), p.getRoom());
		assertEquals(model.getSurface(), p.getSurface());
		assertEquals(-5.0, p.getPosition().getX());
		assertEquals(-5.0, p.getPosition().getY());
		assertEquals(0.0, p.getPosition().getZ());
		assertEquals((Math.PI * 4.0) / 5.0, p.getRotation());
		assertEquals(2.083259, model.getDecisionValue(), 0.000001);
	}
}
