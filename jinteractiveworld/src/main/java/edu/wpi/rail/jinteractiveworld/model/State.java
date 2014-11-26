package edu.wpi.rail.jinteractiveworld.model;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Object;

import java.util.ArrayList;

/**
 * A State contains a collection of Placements in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 12, 2014
 */
public class State {

	private ArrayList<Placement> placements;

	/**
	 * Create a new, empty state.
	 */
	public State() {
		this.placements = new ArrayList<Placement>();
	}

	/**
	 * Add the given object and placement information relative to the global
	 * frame to this state.
	 * 
	 * @param o
	 *            The object to add.
	 * @param r
	 *            The rotation matrix in global coordinates.
	 * @param v
	 *            The displacement vector in global coordinates.
	 */
	public void add(Object o, RotationMatrix r, Vector3 v) {
		this.add(o, new Transform(r, v));
	}

	/**
	 * Add the given object and placement information relative to the global
	 * frame to this state.
	 * 
	 * @param o
	 *            The object to add.
	 * @param tf
	 *            The transform in global coordinates.
	 */
	public void add(Object o, Transform tf) {
		this.add(new Placement(o, tf));
	}

	/**
	 * Add the given placement to this state.
	 * 
	 * @param p
	 *            The placement to add.
	 */
	public void add(Placement p) {
		this.placements.add(p);
	}

	/**
	 * Get a list of all placements in this state.
	 * 
	 * @return A list of all placements in this state.
	 */
	public ArrayList<Placement> getPlacements() {
		return this.placements;
	}

	/**
	 * Get a list of all objects in this state. Note that this may not be a
	 * unique set of objects.
	 * 
	 * @return A list of all objects in this state.
	 */
	public ArrayList<Object> getObjects() {
		ArrayList<Object> objects = new ArrayList<Object>();
		for (Placement p : this.placements) {
			objects.add(p.getObject());
		}
		return objects;
	}

	/**
	 * Get a list of all transforms in this state.
	 * 
	 * @return A list of all transforms in this state.
	 */
	public ArrayList<Transform> getTransforms() {
		ArrayList<Transform> tfs = new ArrayList<Transform>();
		for (Placement p : this.placements) {
			tfs.add(p.getTransform());
		}
		return tfs;
	}
}
