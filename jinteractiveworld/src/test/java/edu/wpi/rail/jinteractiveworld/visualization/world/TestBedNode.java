package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestBedNode {

	@Test
	public void testConstructor() {
		BedNode n = new BedNode();
		assertEquals(2, n.getChildrenCount());
	}
}
