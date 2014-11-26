package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestCouchNode {

	@Test
	public void testConstructor() {
		CouchNode n = new CouchNode();
		assertEquals(2, n.getChildrenCount());
	}
}
