package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestSinkNode {

	@Test
	public void testConstructor() {
		SinkNode n = new SinkNode();
		assertEquals(2, n.getChildrenCount());
	}
}
