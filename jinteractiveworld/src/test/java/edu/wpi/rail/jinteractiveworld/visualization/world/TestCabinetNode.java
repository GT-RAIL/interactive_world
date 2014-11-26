package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestCabinetNode {

	@Test
	public void testConstructor() {
		CabinetNode n = new CabinetNode();
		assertEquals(2, n.getChildrenCount());
	}
}
