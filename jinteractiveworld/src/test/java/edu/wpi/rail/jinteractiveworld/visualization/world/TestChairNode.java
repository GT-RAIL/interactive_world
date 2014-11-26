package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestChairNode {

	@Test
	public void testConstructor() {
		ChairNode n = new ChairNode();
		assertEquals(2, n.getChildrenCount());
	}
}
