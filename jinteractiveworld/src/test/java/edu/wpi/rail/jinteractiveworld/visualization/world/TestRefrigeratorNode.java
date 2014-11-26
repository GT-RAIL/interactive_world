package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestRefrigeratorNode {

	@Test
	public void testConstructor() {
		RefrigeratorNode n = new RefrigeratorNode();
		assertEquals(2, n.getChildrenCount());
	}
}
