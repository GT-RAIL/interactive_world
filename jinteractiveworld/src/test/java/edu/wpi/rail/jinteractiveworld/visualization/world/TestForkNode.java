package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestForkNode {

	@Test
	public void testConstructor() {
		ForkNode n = new ForkNode();
		assertEquals(2, n.getChildrenCount());
	}
}
