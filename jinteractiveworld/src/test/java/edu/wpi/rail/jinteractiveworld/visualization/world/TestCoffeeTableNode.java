package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestCoffeeTableNode {

	@Test
	public void testConstructor() {
		CoffeeTableNode n = new CoffeeTableNode();
		assertEquals(2, n.getChildrenCount());
	}
}
