package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestSpoonNode {

	@Test
	public void testConstructor() {
		SpoonNode n = new SpoonNode();
		assertEquals(2, n.getChildrenCount());
	}
}
