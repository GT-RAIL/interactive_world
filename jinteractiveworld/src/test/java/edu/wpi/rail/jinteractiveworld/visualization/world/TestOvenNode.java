package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestOvenNode {

	@Test
	public void testConstructor() {
		OvenNode n = new OvenNode();
		assertEquals(2, n.getChildrenCount());
	}
}
