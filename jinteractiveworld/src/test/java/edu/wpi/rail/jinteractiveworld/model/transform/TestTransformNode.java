package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

public class TestTransformNode {

	@Test
	public void testFrameAndTfConstructor() {
		String frame = "test";
		TransformNode tfn = new TransformNode(frame, new Transform());

		assertEquals(frame, tfn.getFrame());
		assertEquals(new Transform(), tfn.getTransform());
		assertNull(tfn.getParent());
		assertEquals(0, tfn.size());
	}

	@Test
	public void testFrameTfAndParentConstructor() {
		String frame1 = "test1";
		TransformNode tfn1 = new TransformNode(frame1, new Transform());
		String frame2 = "test2";
		TransformNode tfn2 = new TransformNode(frame2, new Transform(), tfn1);

		assertEquals(frame2, tfn2.getFrame());
		assertEquals(new Transform(), tfn2.getTransform());
		assertEquals(tfn1, tfn2.getParent());
		assertEquals(0, tfn2.size());
	}

	@Test
	public void testSetFrame() {
		String frame1 = "test1";
		TransformNode tfn = new TransformNode(frame1, new Transform());
		String frame2 = "test2";
		tfn.setFrame(frame2);
		assertEquals(frame2, tfn.getFrame());
	}

	@Test
	public void testSetTransform() {
		Transform tf1 = new Transform();
		TransformNode tfn = new TransformNode("test", tf1);
		Transform tf2 = new Transform(new RotationMatrix(),
				new Vector3(1, 2, 3));
		tfn.setTransform(tf2);
		assertEquals(tf2, tfn.getTransform());
	}

	@Test
	public void testSetParent() {
		TransformNode tfn1 = new TransformNode("test1", new Transform());
		TransformNode tfn2 = new TransformNode("test2", new Transform());

		tfn2.setParent(tfn1);
		assertEquals(tfn1, tfn2.getParent());
	}

	@Test
	public void testAddChild() {
		TransformNode tfn0 = new TransformNode("test0", new Transform());
		TransformNode tfn1 = new TransformNode("test1", new Transform());
		TransformNode tfn2 = new TransformNode("test2", new Transform());
		TransformNode tfn3 = new TransformNode("test3", new Transform());
		TransformNode tfn4 = new TransformNode("test4", new Transform());
		TransformNode tfn5 = new TransformNode("test5", new Transform());

		tfn0.addChild(tfn1);
		tfn0.addChild(tfn2);
		tfn2.addChild(tfn3);
		tfn3.addChild(tfn4);
		tfn3.addChild(tfn5);

		assertEquals(5, tfn0.size());
		assertEquals(0, tfn1.size());
		assertEquals(3, tfn2.size());
		assertEquals(2, tfn3.size());
		assertEquals(0, tfn4.size());
		assertEquals(0, tfn5.size());

		ArrayList<TransformNode> tfn0Children = tfn0.getChildren();
		assertEquals(2, tfn0Children.size());
		assertEquals(tfn1, tfn0Children.get(0));
		assertEquals(tfn2, tfn0Children.get(1));
		ArrayList<TransformNode> tfn1Children = tfn1.getChildren();
		assertEquals(0, tfn1Children.size());
		ArrayList<TransformNode> tfn2Children = tfn2.getChildren();
		assertEquals(1, tfn2Children.size());
		assertEquals(tfn3, tfn2Children.get(0));
		ArrayList<TransformNode> tfn3Children = tfn3.getChildren();
		assertEquals(2, tfn3Children.size());
		assertEquals(tfn4, tfn3Children.get(0));
		assertEquals(tfn5, tfn3Children.get(1));
		ArrayList<TransformNode> tfn4Children = tfn4.getChildren();
		assertEquals(0, tfn4Children.size());
		ArrayList<TransformNode> tfn5Children = tfn5.getChildren();
		assertEquals(0, tfn5Children.size());
	}

	@Test
	public void testGetAncestors() {
		TransformNode tfn0 = new TransformNode("test0", new Transform());
		TransformNode tfn1 = new TransformNode("test1", new Transform());
		TransformNode tfn2 = new TransformNode("test2", new Transform());
		TransformNode tfn3 = new TransformNode("test3", new Transform());
		TransformNode tfn4 = new TransformNode("test4", new Transform());
		TransformNode tfn5 = new TransformNode("test5", new Transform());

		tfn0.addChild(tfn1);
		tfn0.addChild(tfn2);
		tfn2.addChild(tfn3);
		tfn3.addChild(tfn4);
		tfn3.addChild(tfn5);
		
		ArrayList<TransformNode> tfn5Ancestors = tfn5.getAncestors();
		assertEquals(3, tfn5Ancestors.size());
		assertEquals(tfn3, tfn5Ancestors.get(0));
		assertEquals(tfn2, tfn5Ancestors.get(1));
		assertEquals(tfn0, tfn5Ancestors.get(2));
	}
}
