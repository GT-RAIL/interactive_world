package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestDiningTableNode {

	@Test
	public void testConstructor() {
		DiningTableNode n = new DiningTableNode();
		assertEquals(2, n.getChildrenCount());
	}
}
