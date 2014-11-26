package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestTVNode {

	@Test
	public void testConstructor() {
		TVNode n = new TVNode();
		assertEquals(2, n.getChildrenCount());
	}
}
