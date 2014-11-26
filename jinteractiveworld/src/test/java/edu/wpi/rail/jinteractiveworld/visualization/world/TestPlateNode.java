package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestPlateNode {

	@Test
	public void testConstructor() {
		PlateNode n = new PlateNode();
		assertEquals(2, n.getChildrenCount());
	}
}
