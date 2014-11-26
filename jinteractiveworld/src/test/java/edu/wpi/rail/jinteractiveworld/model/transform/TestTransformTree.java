package edu.wpi.rail.jinteractiveworld.model.transform;

import static org.junit.Assert.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedList;

import org.junit.Test;

public class TestTransformTree {

	@Test
	public void testConstructor() {
		TransformTree tree = new TransformTree();
		assertEquals(1, tree.size());
		assertEquals(TransformTree.GLOBAL_FRAME,
				tree.findFrame(TransformTree.GLOBAL_FRAME).getFrame());
	}

	@Test
	public void testFindFrameInvalid() {
		TransformTree tree = new TransformTree();
		assertNull(tree.findFrame("invalid"));
	}

	@Test
	public void testAddFrameInvalid() {
		TransformTree tree = new TransformTree();
		assertFalse(tree.addFrame("invalid", "test", new Transform()));
	}

	@Test
	public void testAddFrame() {
		TransformTree tree = new TransformTree();
		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform()));
		assertTrue(tree.addFrame("test0", "test2", new Transform()));
		assertTrue(tree.addFrame("test2", "test3", new Transform()));
		assertTrue(tree.addFrame("test3", "test4", new Transform()));
		assertTrue(tree.addFrame("test3", "test5", new Transform()));

		assertEquals(7, tree.size());

		TransformNode tfnGlobal = tree.findFrame(TransformTree.GLOBAL_FRAME);
		TransformNode tfn0 = tree.findFrame("test0");
		TransformNode tfn1 = tree.findFrame("test1");
		TransformNode tfn2 = tree.findFrame("test2");
		TransformNode tfn3 = tree.findFrame("test3");
		TransformNode tfn4 = tree.findFrame("test4");
		TransformNode tfn5 = tree.findFrame("test5");

		assertEquals(6, tfnGlobal.size());
		assertEquals(TransformTree.GLOBAL_FRAME, tfnGlobal.getFrame());
		assertNull(tfnGlobal.getParent());
		assertEquals(5, tfn0.size());
		assertEquals("test0", tfn0.getFrame());
		assertEquals(tfnGlobal, tfn0.getParent());
		assertEquals(0, tfn1.size());
		assertEquals("test1", tfn1.getFrame());
		assertEquals(tfn0, tfn1.getParent());
		assertEquals(3, tfn2.size());
		assertEquals("test2", tfn2.getFrame());
		assertEquals(tfn0, tfn2.getParent());
		assertEquals(2, tfn3.size());
		assertEquals("test3", tfn3.getFrame());
		assertEquals(tfn2, tfn3.getParent());
		assertEquals(0, tfn4.size());
		assertEquals("test4", tfn4.getFrame());
		assertEquals(tfn3, tfn4.getParent());
		assertEquals(0, tfn5.size());
		assertEquals("test5", tfn5.getFrame());
		assertEquals(tfn3, tfn5.getParent());

		ArrayList<TransformNode> tfnGlobalChildren = tfnGlobal.getChildren();
		assertEquals(1, tfnGlobalChildren.size());
		assertEquals(tfn0, tfnGlobalChildren.get(0));
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
	public void testUpdateFrameInvalid() {
		TransformTree tft = new TransformTree();

		assertFalse(tft.updateFrame("invalid", new Transform()));
	}

	@Test
	public void testUpdateFrameSimpleCase() {
		TransformTree tree = new TransformTree();

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform()));
		assertTrue(tree.addFrame("test0", "test2", new Transform()));
		assertTrue(tree.addFrame("test2", "test3", new Transform()));
		assertTrue(tree.addFrame("test3", "test4", new Transform()));
		assertTrue(tree.addFrame("test3", "test5", new Transform()));

		Transform newTf = new Transform(new RotationMatrix(Math.PI,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(1, 2, 3));

		assertTrue(tree.updateFrame("test0", newTf));
		assertEquals(newTf, tree.findFrame("test0").getTransform());
		assertTrue(tree.updateFrame("test1", newTf));
		assertEquals(newTf, tree.findFrame("test1").getTransform());
		assertTrue(tree.updateFrame("test2", newTf));
		assertEquals(newTf, tree.findFrame("test2").getTransform());
		assertTrue(tree.updateFrame("test3", newTf));
		assertEquals(newTf, tree.findFrame("test3").getTransform());
		assertTrue(tree.updateFrame("test4", newTf));
		assertEquals(newTf, tree.findFrame("test4").getTransform());
		assertTrue(tree.updateFrame("test5", newTf));
		assertEquals(newTf, tree.findFrame("test5").getTransform());
	}

	@Test
	public void testGetTransformInvalidReference() {
		TransformTree tree = new TransformTree();
		assertNull(tree.getTransform("invalid", TransformTree.GLOBAL_FRAME));
	}

	@Test
	public void testGetTransformInvalidTarget() {
		TransformTree tree = new TransformTree();
		assertNull(tree.getTransform(TransformTree.GLOBAL_FRAME, "invalid"));
	}

	@Test
	public void testGetTransformSimpleCase() {
		double theta = Math.PI / 2.0;
		double x = 0.1;
		double y = 0.2;
		double z = 0.5;

		TransformTree tree = new TransformTree();
		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform(
				new RotationMatrix(theta,
						RotationMatrix.RotationType.Z_ROTATION), new Vector3(x,
						y, z))));

		assertEquals(new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z)),
				tree.getTransform("test0", "test1"));
	}

	@Test
	public void testGetTransformInverseCase() {
		double theta = Math.PI / 2.0;
		double x = 0.1;
		double y = 0.2;
		double z = 0.5;

		TransformTree tree = new TransformTree();
		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform(
				new RotationMatrix(theta,
						RotationMatrix.RotationType.Z_ROTATION), new Vector3(x,
						y, z))));

		Transform toInvert = new Transform(new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(x, y, z));
		assertEquals(toInvert.getInverse(), tree.getTransform("test1", "test0"));
	}

	@Test
	public void testGetTransformSimplePath() {
		TransformTree tree = new TransformTree();

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "A",
				new Transform(new RotationMatrix(Math.PI,
						RotationMatrix.RotationType.X_ROTATION), new Vector3(
						-2, -5, 10))));
		assertTrue(tree.addFrame("A", "B", new Transform(new RotationMatrix(
				Math.PI, RotationMatrix.RotationType.Z_ROTATION), new Vector3(
				-2, 0, 0))));
		assertTrue(tree.addFrame("B", "C", new Transform(new RotationMatrix(
				-Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-4, -1, 0))));
		assertTrue(tree.addFrame("B", "D", new Transform(new RotationMatrix(
				Math.PI / 4.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-12, 5, 1))));
		assertTrue(tree.addFrame("A", "E", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-0.2, 4, 2))));

		Transform tf = tree.getTransform("A", "C");
		assertEquals(0, tf.getR11(), 0.0000000000000001);
		assertEquals(-1, tf.getR12(), 0);
		assertEquals(0, tf.getR13(), 0);
		assertEquals(2, tf.getR14(), 0);
		assertEquals(1, tf.getR21(), 0);
		assertEquals(0, tf.getR22(), 0.0000000000000001);
		assertEquals(0, tf.getR23(), 0);
		assertEquals(1, tf.getR24(), 0.000000000000001);
		assertEquals(0, tf.getR31(), 0);
		assertEquals(0, tf.getR32(), 0);
		assertEquals(1, tf.getR33(), 0);
		assertEquals(0, tf.getR34(), 0);
		assertEquals(0, tf.getR41(), 0);
		assertEquals(0, tf.getR42(), 0);
		assertEquals(0, tf.getR43(), 0);
		assertEquals(1, tf.getR44(), 0);
	}

	@Test
	public void testGetTransformSimplePathInverse() {
		TransformTree tree = new TransformTree();

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "A",
				new Transform(new RotationMatrix(Math.PI,
						RotationMatrix.RotationType.X_ROTATION), new Vector3(
						-2, -5, 10))));
		assertTrue(tree.addFrame("A", "B", new Transform(new RotationMatrix(
				Math.PI, RotationMatrix.RotationType.Z_ROTATION), new Vector3(
				-2, 0, 0))));
		assertTrue(tree.addFrame("B", "C", new Transform(new RotationMatrix(
				-Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-4, -1, 0))));
		assertTrue(tree.addFrame("B", "D", new Transform(new RotationMatrix(
				Math.PI / 4.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-12, 5, 1))));
		assertTrue(tree.addFrame("A", "E", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-0.2, 4, 2))));

		Transform tf = tree.getTransform("C", "A");
		assertEquals(0, tf.getR11(), 0.0000000000000001);
		assertEquals(1, tf.getR12(), 0);
		assertEquals(0, tf.getR13(), 0);
		assertEquals(-1, tf.getR14(), 0.000000000000001);
		assertEquals(-1, tf.getR21(), 0);
		assertEquals(0, tf.getR22(), 0.0000000000000001);
		assertEquals(0, tf.getR23(), 0);
		assertEquals(2, tf.getR24(), 0);
		assertEquals(0, tf.getR31(), 0);
		assertEquals(0, tf.getR32(), 0);
		assertEquals(1, tf.getR33(), 0);
		assertEquals(0, tf.getR34(), 0);
		assertEquals(0, tf.getR41(), 0);
		assertEquals(0, tf.getR42(), 0);
		assertEquals(0, tf.getR43(), 0);
		assertEquals(1, tf.getR44(), 0);
	}

	@Test
	public void testGetTransform() {
		TransformTree tree = new TransformTree();

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "A",
				new Transform(new RotationMatrix(Math.PI,
						RotationMatrix.RotationType.X_ROTATION), new Vector3(
						-2, -5, 10))));
		assertTrue(tree.addFrame("A", "B", new Transform(new RotationMatrix(
				Math.PI, RotationMatrix.RotationType.Z_ROTATION), new Vector3(
				-2, 0, 0))));
		assertTrue(tree.addFrame("A", "C", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(2, 1, 0))));
		assertTrue(tree.addFrame("B", "D", new Transform()));
		assertTrue(tree.addFrame("A", "E", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-0.2, 4, 2))));
		assertTrue(tree.addFrame("C", "F", new Transform()));

		Transform tf = tree.getTransform("D", "F");
		assertEquals(0, tf.getR11(), 0.000000000000001);
		assertEquals(1, tf.getR12(), 0);
		assertEquals(0, tf.getR13(), 0);
		assertEquals(-4, tf.getR14(), 0);
		assertEquals(-1, tf.getR21(), 0);
		assertEquals(0, tf.getR22(), 0.000000000000001);
		assertEquals(0, tf.getR23(), 0);
		assertEquals(-1, tf.getR24(), 0.000000000000001);
		assertEquals(0, tf.getR31(), 0);
		assertEquals(0, tf.getR32(), 0);
		assertEquals(1, tf.getR33(), 0);
		assertEquals(0, tf.getR34(), 0);
		assertEquals(0, tf.getR41(), 0);
		assertEquals(0, tf.getR42(), 0);
		assertEquals(0, tf.getR43(), 0);
		assertEquals(1, tf.getR44(), 0);
	}

	@Test
	public void testGetTransform2() {
		TransformTree tree = new TransformTree();

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "A",
				new Transform(new RotationMatrix(Math.PI,
						RotationMatrix.RotationType.X_ROTATION), new Vector3(
						-2, -5, 10))));
		assertTrue(tree.addFrame("A", "B", new Transform(new RotationMatrix(
				Math.PI, RotationMatrix.RotationType.Z_ROTATION), new Vector3(
				-2, 0, 0))));
		assertTrue(tree.addFrame("A", "C", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(2, 1, 0))));
		assertTrue(tree.addFrame("B", "D", new Transform()));
		assertTrue(tree.addFrame("A", "E", new Transform(new RotationMatrix(
				Math.PI / 2.0, RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(-0.2, 4, 2))));
		assertTrue(tree.addFrame("C", "F", new Transform()));

		Transform tf = tree.getTransform("F", "D");
		assertEquals(0, tf.getR11(), 0.0000000000000001);
		assertEquals(-1, tf.getR12(), 0);
		assertEquals(0, tf.getR13(), 0);
		assertEquals(-1, tf.getR14(), 0.000000000000001);
		assertEquals(1, tf.getR21(), 0);
		assertEquals(0, tf.getR22(), 0.0000000000000001);
		assertEquals(0, tf.getR23(), 0);
		assertEquals(4, tf.getR24(), 0);
		assertEquals(0, tf.getR31(), 0);
		assertEquals(0, tf.getR32(), 0);
		assertEquals(1, tf.getR33(), 0);
		assertEquals(0, tf.getR34(), 0);
		assertEquals(0, tf.getR41(), 0);
		assertEquals(0, tf.getR42(), 0);
		assertEquals(0, tf.getR43(), 0);
		assertEquals(1, tf.getR44(), 0);
	}

	@Test
	public void testSearchDown() throws NoSuchMethodException,
			SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {
		TransformTree tree = new TransformTree();

		Method method = tree.getClass().getDeclaredMethod("searchDown",
				TransformNode.class, String.class, LinkedList.class);
		method.setAccessible(true);

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform()));
		assertTrue(tree.addFrame("test0", "test2", new Transform()));
		assertTrue(tree.addFrame("test2", "test3", new Transform()));
		assertTrue(tree.addFrame("test3", "test4", new Transform()));
		assertTrue(tree.addFrame("test3", "test5", new Transform()));

		@SuppressWarnings("unchecked")
		LinkedList<TransformNode> path = ((LinkedList<TransformNode>) method
				.invoke(tree, tree.findFrame(TransformTree.GLOBAL_FRAME),
						"test5", new LinkedList<TransformNode>()));
		assertEquals(5, path.size());
		assertEquals(TransformTree.GLOBAL_FRAME, path.get(0).getFrame());
		assertEquals("test0", path.get(1).getFrame());
		assertEquals("test2", path.get(2).getFrame());
		assertEquals("test3", path.get(3).getFrame());
		assertEquals("test5", path.get(4).getFrame());
	}

	@Test
	public void testSearchDownInvalid() throws NoSuchMethodException,
			SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {
		TransformTree tree = new TransformTree();

		Method method = tree.getClass().getDeclaredMethod("searchDown",
				TransformNode.class, String.class, LinkedList.class);
		method.setAccessible(true);

		assertTrue(tree.addFrame(TransformTree.GLOBAL_FRAME, "test0",
				new Transform()));
		assertTrue(tree.addFrame("test0", "test1", new Transform()));
		assertTrue(tree.addFrame("test0", "test2", new Transform()));
		assertTrue(tree.addFrame("test2", "test3", new Transform()));
		assertTrue(tree.addFrame("test3", "test4", new Transform()));
		assertTrue(tree.addFrame("test3", "test5", new Transform()));

		@SuppressWarnings("unchecked")
		LinkedList<TransformNode> path = ((LinkedList<TransformNode>) method
				.invoke(tree, tree.findFrame("test1"), "test5",
						new LinkedList<TransformNode>()));
		assertNull(path);
	}
}
