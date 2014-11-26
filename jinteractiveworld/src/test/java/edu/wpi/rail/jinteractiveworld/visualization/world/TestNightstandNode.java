package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestNightstandNode {

	@Test
	public void testConstructor() {
		NightstandNode n = new NightstandNode();
		assertEquals(2, n.getChildrenCount());
	}
}
