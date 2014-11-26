package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestPillowNode {

	@Test
	public void testConstructor() {
		PillowNode n = new PillowNode();
		assertEquals(2, n.getChildrenCount());
	}
}
