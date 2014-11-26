package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestBurnerNode {

	@Test
	public void testConstructor() {
		BurnerNode n = new BurnerNode();
		assertEquals(2, n.getChildrenCount());
	}
}
