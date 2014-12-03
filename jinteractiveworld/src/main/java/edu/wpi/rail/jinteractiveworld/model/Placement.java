package edu.wpi.rail.jinteractiveworld.model;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A Placement is a pairing between a transform and an object.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 12, 2014
 */
public class Placement {

	private Object o;
	private Transform tf;

	/**
	 * Create a new placement based on the given object, rotation matrix, and
	 * displacement vector.
	 * 
	 * @param o
	 *            The object for this placement.
	 * @param r
	 *            The rotation matrix of the object in global coordinates.
	 * @param v
	 *            The displacement vector of the object in global coordinates.
	 */
	public Placement(Object o, RotationMatrix r, Vector3 v) {
		this(o, new Transform(r, v));
	}

	/**
	 * Create a new placement based on the given object and transform in global
	 * coordinates.
	 * 
	 * @param o
	 *            The object for this placement.
	 * @param tf
	 *            The transform in global coordinates.
	 */
	public Placement(Object o, Transform tf) {
		this.o = o;
		this.tf = tf;
	}

	/**
	 * Get the object used in this placement.
	 * 
	 * @return The object used in this placement.
	 */
	public Object getObject() {
		return this.o;
	}

	/**
	 * Set the object for this placement.
	 * 
	 * @param o
	 *            The object for this placement.
	 */
	public void setObject(Object o) {
		this.o = o;
	}

	/**
	 * Get the transform for this placement in global coordinates.
	 * 
	 * @return The transform for this placement in global coordinates.
	 */
	public Transform getTransform() {
		return this.tf;
	}

	/**
	 * Set the transform for this placement in global coordinates.
	 * 
	 * @param tf
	 *            The transform for this placement in global coordinates.
	 */
	public void setTransform(Transform tf) {
		this.tf = tf;
	}

	/**
	 * Check if two Placements are equal. Two Placements are equal if their
	 * Objects and Transforms are equal.
	 * 
	 * @param o
	 *            The java.lang.Item to test.
	 * @return If the given java.lang.Item is equal to this Placement.
	 */
	@Override
	public boolean equals(java.lang.Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof Placement) {
			Placement p = (Placement) o;
			return p.getObject().equals(this.o)
					&& p.getTransform().equals(this.tf);
		} else {
			return false;
		}
	}
}
