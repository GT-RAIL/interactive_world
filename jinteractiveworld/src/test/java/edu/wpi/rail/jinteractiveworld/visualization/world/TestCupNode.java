package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestCupNode {

	@Test
	public void testConstructor() {
		CupNode n = new CupNode();
		assertEquals(2, n.getChildrenCount());
	}
}
