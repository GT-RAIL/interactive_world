package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestDresserNode {

	@Test
	public void testConstructor() {
		DresserNode n = new DresserNode();
		assertEquals(2, n.getChildrenCount());
	}
}
