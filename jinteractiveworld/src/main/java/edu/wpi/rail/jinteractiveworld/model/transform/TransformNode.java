package edu.wpi.rail.jinteractiveworld.model.transform;

import java.util.ArrayList;

/**
 * The TransformNode represents a node in the transform tree. It contains a
 * single parent and an arbitrary number of children.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 12, 2014
 */
public class TransformNode {

	private String frame;
	private Transform tf;
	private TransformNode parent;
	private ArrayList<TransformNode> children;

	/**
	 * Create a node with the given frame name and initial transform. The parent
	 * frame will be null meaning the root of a tree.
	 * 
	 * @param frame
	 *            The unique name of this frame in the tree.
	 * @param tf
	 *            The transform for this frame relative to its parent.
	 */
	public TransformNode(String frame, Transform tf) {
		this(frame, tf, null);
	}

	/**
	 * Create a node with the given frame name, initial transform, and parent.
	 * 
	 * 
	 * @param frame
	 *            The unique name of this frame in the tree.
	 * @param tf
	 *            The transform for this frame relative to its parent.
	 * @param parent
	 *            The parent node.
	 */
	public TransformNode(String frame, Transform tf, TransformNode parent) {
		this.frame = frame;
		this.tf = tf;
		this.parent = parent;
		this.children = new ArrayList<TransformNode>();
	}

	/**
	 * Get the frame name of this node.
	 * 
	 * @return The frame name of this node.
	 */
	public String getFrame() {
		return this.frame;
	}

	/**
	 * Set the frame name of this node.
	 * 
	 * @param frame
	 *            The frame name of this node.
	 */
	public void setFrame(String frame) {
		this.frame = frame;
	}

	/**
	 * Get the transform relative to this frames parent.
	 * 
	 * @return The transform relative to this frames parent.
	 */
	public Transform getTransform() {
		return this.tf;
	}

	/**
	 * Set the transform relative to its parent.
	 * 
	 * @param tf
	 *            The transform relative to its parent.
	 */
	public void setTransform(Transform tf) {
		this.tf = tf;
	}

	/**
	 * Get the parent node of this node.
	 * 
	 * @return The parent node of this node.
	 */
	public TransformNode getParent() {
		return this.parent;
	}

	/**
	 * Set the parent node.
	 * 
	 * @param parent
	 *            The parent node.
	 */
	public void setParent(TransformNode parent) {
		this.parent = parent;
	}

	/**
	 * Get a list of all ancestors of this node. An empty list is returned if
	 * there are none.
	 * 
	 * @return A list of all ancestors of this node.
	 */
	public ArrayList<TransformNode> getAncestors() {
		ArrayList<TransformNode> ancestors = new ArrayList<TransformNode>();
		TransformNode current = this.parent;
		// fill the list
		while (current != null) {
			ancestors.add(current);
			current = current.getParent();
		}

		return ancestors;
	}

	/**
	 * Get a list of all children of this node. An empty list is returned if
	 * there are none.
	 * 
	 * @return A list of all children of this node.
	 */
	public ArrayList<TransformNode> getChildren() {
		return this.children;
	}

	/**
	 * Add a child to this node. The given node's parent is set to this node.
	 * 
	 * @param node
	 *            The node to add as a child.
	 */
	public void addChild(TransformNode node) {
		this.children.add(node);
		node.setParent(this);
	}

	/**
	 * Get the number of children of this node.
	 * 
	 * @return The number of children of this node.
	 */
	public int size() {
		int total = this.children.size();
		for (int i = 0; i < this.children.size(); i++) {
			total += this.children.get(i).size();
		}
		return total;
	}
}
