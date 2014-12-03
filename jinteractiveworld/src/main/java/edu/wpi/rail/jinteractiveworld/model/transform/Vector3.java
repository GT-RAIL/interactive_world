package edu.wpi.rail.jinteractiveworld.model.transform;

import org.ejml.data.FixedMatrix3_64F;

/**
 * A Vector3 represents a 3-dimensional vector with x, y, and z components.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 6, 2014
 */
public class Vector3 {

	// internal storage
	private FixedMatrix3_64F v;

	/**
	 * Create a new vector with all 0s.
	 */
	public Vector3() {
		this(0);
	}

	/**
	 * Create a new vector with the given x value (y and z values will 0).
	 * 
	 * @param x
	 *            The x value of the vector.
	 */
	public Vector3(double x) {
		this(x, 0);
	}

	/**
	 * Create a new vector with the given x and y values (the z value will be
	 * set to 0).
	 * 
	 * @param x
	 *            The x value of the vector.
	 * @param y
	 *            The y value of the vector.
	 */
	public Vector3(double x, double y) {
		this(x, y, 0);
	}

	/**
	 * Create a new vector with the given values.
	 * 
	 * @param x
	 *            The x value of the vector.
	 * @param y
	 *            The y value of the vector.
	 * @param z
	 *            The z value of the vector.
	 */
	public Vector3(double x, double y, double z) {
		this.v = new FixedMatrix3_64F(x, y, z);
	}

	/**
	 * Get the x value of this vector.
	 * 
	 * @return The x value of this vector.
	 */
	public double getX() {
		return this.v.a1;
	}

	/**
	 * Set the x value of this vector.
	 * 
	 * @param x
	 *            The x value of this vector.
	 */
	public void setX(double x) {
		this.v.a1 = x;
	}

	/**
	 * Get the y value of this vector.
	 * 
	 * @return The y value of this vector.
	 */
	public double getY() {
		return this.v.a2;
	}

	/**
	 * Set the x value of this vector.
	 * 
	 * @param y
	 *            The y value of this vector.
	 */
	public void setY(double y) {
		this.v.a2 = y;
	}

	/**
	 * Get the z value of this vector.
	 * 
	 * @return The z value of this vector.
	 */
	public double getZ() {
		return this.v.a3;
	}

	/**
	 * Set the z value of this vector.
	 * 
	 * @param z
	 *            The z value of this vector.
	 */
	public void setZ(double z) {
		this.v.a3 = z;
	}

	/**
	 * Check if the given Item is the same as the vector. Two vectors are the
	 * same if their x, y, and z values are equal.
	 * 
	 * @param o
	 *            The Item to check.
	 * @return If the Item is equal to this Vector3.
	 */
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof Vector3) {
			Vector3 p = (Vector3) o;
			return p.getX() == this.getX() && p.getY() == this.getY()
					&& p.getZ() == this.getZ();
		} else {
			return false;
		}
	}

	/**
	 * Create a clone of this Vector3.
	 * 
	 * @return A clone of this Vector3.
	 */
	public Vector3 clone() {
		return new Vector3(this.getX(), this.getY(), this.getZ());
	}
}
