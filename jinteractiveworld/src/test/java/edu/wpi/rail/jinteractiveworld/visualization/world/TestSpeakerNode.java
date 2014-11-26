package edu.wpi.rail.jinteractiveworld.visualization.world;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestSpeakerNode {

	@Test
	public void testConstructor() {
		SpeakerNode n = new SpeakerNode();
		assertEquals(2, n.getChildrenCount());
	}
}
