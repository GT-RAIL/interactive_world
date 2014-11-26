package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestCounterNode {

	@Test
	public void testConstructor() {
		CounterNode n = new CounterNode();
		assertEquals(2, n.getChildrenCount());
	}
}
