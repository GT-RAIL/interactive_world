package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestMagazinesNode {

	@Test
	public void testConstructor() {
		MagazinesNode n = new MagazinesNode();
		assertEquals(2, n.getChildrenCount());
	}
}
