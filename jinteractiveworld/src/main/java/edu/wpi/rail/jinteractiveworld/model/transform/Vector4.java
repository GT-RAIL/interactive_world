package edu.wpi.rail.jinteractiveworld.model.transform;

import org.ejml.data.FixedMatrix4_64F;

/**
 * A Vector4 represents a 3-dimensional vector with w, x, y, and z components.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version August 14, 2014
 */
public class Vector4 {

	// internal storage
	private FixedMatrix4_64F v;

	/**
	 * Create a new vector with all 0s.
	 */
	public Vector4() {
		this(0);
	}

	/**
	 * Create a new vector with the given w value (x, y, and z values will 0).
	 * 
	 * @param w
	 *            The w value of the vector.
	 */
	public Vector4(double w) {
		this(w, 0);
	}

	/**
	 * Create a new vector with the given w and x values (the y and z value will
	 * be set to 0).
	 * 
	 * @param w
	 *            The x value of the vector.
	 * @param x
	 *            The x value of the vector.
	 */
	public Vector4(double w, double x) {
		this(w, x, 0);
	}

	/**
	 * Create a new vector with the given values.
	 * 
	 * @param w
	 *            The w value of the vector.
	 * @param x
	 *            The x value of the vector.
	 * @param y
	 *            The y value of the vector.
	 */
	public Vector4(double w, double x, double y) {
		this(w, x, y, 0);
	}

	/**
	 * Create a new vector with the given values.
	 * 
	 * @param w
	 *            The w value of the vector.
	 * @param x
	 *            The x value of the vector.
	 * @param y
	 *            The y value of the vector.
	 * @param z
	 *            The z value of the vector.
	 */
	public Vector4(double w, double x, double y, double z) {
		this.v = new FixedMatrix4_64F(w, x, y, z);
	}

	/**
	 * Get the w value of this vector.
	 * 
	 * @return The w value of this vector.
	 */
	public double getW() {
		return this.v.a1;
	}

	/**
	 * Set the w value of this vector.
	 * 
	 * @param w
	 *            The w value of this vector.
	 */
	public void setW(double w) {
		this.v.a1 = w;
	}

	/**
	 * Get the x value of this vector.
	 * 
	 * @return The x value of this vector.
	 */
	public double getX() {
		return this.v.a2;
	}

	/**
	 * Set the x value of this vector.
	 * 
	 * @param x
	 *            The x value of this vector.
	 */
	public void setX(double x) {
		this.v.a2 = x;
	}

	/**
	 * Get the y value of this vector.
	 * 
	 * @return The y value of this vector.
	 */
	public double getY() {
		return this.v.a3;
	}

	/**
	 * Set the y value of this vector.
	 * 
	 * @param y
	 *            The y value of this vector.
	 */
	public void setY(double y) {
		this.v.a3 = y;
	}

	/**
	 * Get the z value of this vector.
	 * 
	 * @return The z value of this vector.
	 */
	public double getZ() {
		return this.v.a4;
	}

	/**
	 * Set the z value of this vector.
	 * 
	 * @param z
	 *            The z value of this vector.
	 */
	public void setZ(double z) {
		this.v.a4 = z;
	}

	/**
	 * Check if the given Object is the same as the vector. Two vectors are the
	 * same if their x, y, and z values are equal.
	 * 
	 * @param o
	 *            The Object to check.
	 * @return If the Object is equal to this Vector4.
	 */
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof Vector4) {
			Vector4 p = (Vector4) o;
			return p.getW() == this.getW() && p.getX() == this.getX()
					&& p.getY() == this.getY() && p.getZ() == this.getZ();
		} else {
			return false;
		}
	}

	/**
	 * Create a clone of this Vector4.
	 * 
	 * @return A clone of this Vector4.
	 */
	public Vector4 clone() {
		return new Vector4(this.getW(), this.getX(), this.getY(), this.getZ());
	}
}
