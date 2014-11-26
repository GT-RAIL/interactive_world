package edu.wpi.rail.jinteractiveworld.model.transform;

import java.util.ArrayList;
import java.util.LinkedList;

/**
 * The TransformTree represents a tree of transforms relative to each other.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 12, 2014
 */
public class TransformTree {

	public static final String GLOBAL_FRAME = "global";

	private TransformNode root;

	/**
	 * Create a new transform tree with a single global frame added as the root
	 * node.
	 */
	public TransformTree() {
		// global frame
		Transform tf = new Transform();
		this.root = new TransformNode(TransformTree.GLOBAL_FRAME, tf);
	}

	/**
	 * Return the number of nodes in this tree.
	 * 
	 * @return The number of nodes in this tree.
	 */
	public int size() {
		return this.root.size() + 1;
	}

	/**
	 * Add a frame to the tree. If the given parent frame does not exist in the
	 * tree, no effect is made and false is returned.
	 * 
	 * @param parentFrame
	 *            The name of the parent frame.
	 * @param frame
	 *            The name of the new frame.
	 * @param tf
	 *            The transform of the new frame relative to the parent.
	 * @return If the change was made.
	 */
	public boolean addFrame(String parentFrame, String frame, Transform tf) {
		// search for the parent
		TransformNode parent = this.findFrame(parentFrame);
		if (parent != null) {
			parent.addChild(new TransformNode(frame, tf, parent));
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Find the given frame in the tree.
	 * 
	 * @param frame
	 *            The frame to find.
	 * @return The node for the given frame, or null if no frame was found.
	 */
	public TransformNode findFrame(String frame) {
		// start with a queue to check
		LinkedList<TransformNode> queue = new LinkedList<TransformNode>();
		// add the root first
		queue.add(root);

		// go until we run out of nodes
		while (!queue.isEmpty()) {
			TransformNode toCheck = queue.pop();
			if (toCheck.getFrame().equals(frame)) {
				return toCheck;
			} else {
				queue.addAll(toCheck.getChildren());
			}
		}

		// none found
		return null;
	}

	/**
	 * Get the transform between the two frames.
	 * 
	 * @param referenceFrame
	 *            The reference frame name.
	 * @param frame
	 *            The name of the target frame.
	 * @return The transform between the two, or null if none was found.
	 */
	public Transform getTransform(String referenceFrame, String frame) {
		// search for the frames
		TransformNode reference = this.findFrame(referenceFrame);
		TransformNode target = this.findFrame(frame);
		if (reference != null && target != null) {
			// check for direct paths
			LinkedList<TransformNode> path1 = this.searchDown(reference, frame,
					new LinkedList<TransformNode>());
			LinkedList<TransformNode> path2 = this.searchDown(target,
					referenceFrame, new LinkedList<TransformNode>());

			if (path1 != null) {
				// start in this coordinate frame
				path1.pop();
				// multiple the rest of the path
				Transform result = path1.pop().getTransform();
				while (!path1.isEmpty()) {
					result = result.multiply(path1.pop().getTransform());
				}
				return result;
			} else if (path2 != null) {
				// return the inverse
				return this.getTransform(frame, referenceFrame).getInverse();
			} else {
				// find the common ancestor
				ArrayList<TransformNode> referenceAncestors = reference
						.getAncestors();
				ArrayList<TransformNode> targetAncestors = target
						.getAncestors();
				Transform result = null;
				// search for the first common ancestor
				for (int i = 0; i < referenceAncestors.size(); i++) {
					TransformNode ancestor = referenceAncestors.get(i);
					if (targetAncestors.contains(ancestor)) {
						// start with the ancestor relative to the reference
						Transform tReferenceAncestor = this.getTransform(
								referenceFrame, ancestor.getFrame());
						// now get the target relative to the ancestor
						Transform tAncestorTarget = this.getTransform(
								ancestor.getFrame(), frame);
						// multiply and return
						result = tReferenceAncestor.multiply(tAncestorTarget);
						i = referenceAncestors.size();
					}
				}
				return result;
			}
		}
		// one or more of the nodes do not exist
		return null;
	}

	/**
	 * Search down the tree starting from the given node for a path to the
	 * target.
	 * 
	 * @param node
	 *            The node to search from.
	 * @param target
	 *            The target frame to look for.
	 * @param path
	 *            The current path.
	 * @return The path to the node or null if one does not exist.
	 */
	private LinkedList<TransformNode> searchDown(TransformNode node,
			String target, LinkedList<TransformNode> path) {
		// base case
		if (node.getFrame().equals(target)) {
			path.add(node);
			return path;
		} else {
			ArrayList<TransformNode> children = node.getChildren();

			// search the children
			path.add(node);
			for (TransformNode n : children) {
				LinkedList<TransformNode> finalPath = this.searchDown(n,
						target, path);
				if (finalPath != null) {
					return finalPath;
				}
			}
			path.removeLast();

			// path not found
			return null;
		}
	}

	/**
	 * Update the given frame relative to its parent.
	 * 
	 * @param frame
	 *            The frame name to update.
	 * @param tf
	 *            The new transform relative to its parent.
	 * @return If a change was made, or null if the frame does not exist.
	 */
	public boolean updateFrame(String frame, Transform tf) {
		// check for the frame first
		TransformNode tfn = this.findFrame(frame);
		if (tfn != null) {
			tfn.setTransform(tf);
			return true;
		} else {
			// no frame found
			return false;
		}
	}
}
