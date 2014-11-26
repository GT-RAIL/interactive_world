package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestSinkUnitNode {

	@Test
	public void testConstructor() {
		SinkUnitNode n = new SinkUnitNode();
		assertEquals(2, n.getChildrenCount());
	}
}
