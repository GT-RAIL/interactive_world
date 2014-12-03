package edu.wpi.rail.jinteractiveworld.model.transform;

import org.ejml.data.FixedMatrix3x3_64F;

/**
 * A RotationMatrix represents a 3-dimensional rotation as a matrix.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version August 14, 2014
 */
public class RotationMatrix {

	/**
	 * The RotationType defines the axes a rotation can be set relative to.
	 * 
	 * @author Russell Toris -- rctoris@wpi.edu
	 * @version February 6, 2014
	 */
	public enum RotationType {
		X_ROTATION, Y_ROTATION, Z_ROTATION
	}

	// internal storage
	private FixedMatrix3x3_64F m;

	/**
	 * Create a rotation matrix with 0 rotation.
	 */
	public RotationMatrix() {
		this(0, RotationMatrix.RotationType.Z_ROTATION);
	}

	/**
	 * Create a rotation matrix with the given theta about the given axis.
	 * 
	 * @param theta
	 *            The rotation (in radians).
	 * @param type
	 *            The axis this rotation is relative to.
	 */
	public RotationMatrix(double theta, RotationMatrix.RotationType type) {
		this.setRotation(theta, type);
	}

	/**
	 * Create a RotationMatrix based on the given components.
	 * 
	 * @param r11
	 *            The R11 component of the matrix.
	 * @param r12
	 *            The R12 component of the matrix.
	 * @param r13
	 *            The R13 component of the matrix.
	 * @param r21
	 *            The R21 component of the matrix.
	 * @param r22
	 *            The R22 component of the matrix.
	 * @param r23
	 *            The R23 component of the matrix.
	 * @param r31
	 *            The R31 component of the matrix.
	 * @param r32
	 *            The R32 component of the matrix.
	 * @param r33
	 *            The R33 component of the matrix.
	 */
	public RotationMatrix(double r11, double r12, double r13, double r21,
			double r22, double r23, double r31, double r32, double r33) {
		this.m = new FixedMatrix3x3_64F(r11, r12, r13, r21, r22, r23, r31, r32,
				r33);
	}

	/**
	 * Set the rotation with the given theta about the given axis.
	 * 
	 * @param theta
	 *            The rotation (in radians).
	 * @param type
	 *            The axis this rotation is relative to.
	 */
	public void setRotation(double theta, RotationMatrix.RotationType type) {
		// precompute the sin/cos
		double sin = Math.sin(theta);
		double cos = Math.cos(theta);

		// determine the matrix type
		if (type == RotationType.X_ROTATION) {
			this.m = new FixedMatrix3x3_64F(1, 0, 0, 0, cos, -sin, 0, sin, cos);
		} else if (type == RotationType.Y_ROTATION) {
			this.m = new FixedMatrix3x3_64F(cos, 0, sin, 0, 1, 0, -sin, 0, cos);
		} else {
			this.m = new FixedMatrix3x3_64F(cos, -sin, 0, sin, cos, 0, 0, 0, 1);
		}
	}

	/**
	 * Get the R11 component of the matrix.
	 * 
	 * @return The R11 component of the matrix.
	 */
	public double getR11() {
		return this.m.a11;
	}

	/**
	 * Set the R11 component of the matrix.
	 * 
	 * @param r11
	 *            The R11 component of the matrix.
	 */
	public void setR11(double r11) {
		this.m.a11 = r11;
	}

	/**
	 * Get the R12 component of the matrix.
	 * 
	 * @return The R12 component of the matrix.
	 */
	public double getR12() {
		return this.m.a12;
	}

	/**
	 * Set the R12 component of the matrix.
	 * 
	 * @param r12
	 *            The R12 component of the matrix.
	 */
	public void setR12(double r12) {
		this.m.a12 = r12;
	}

	/**
	 * Get the R13 component of the matrix.
	 * 
	 * @return The R13 component of the matrix.
	 */
	public double getR13() {
		return this.m.a13;
	}

	/**
	 * Set the R13 component of the matrix.
	 * 
	 * @param r13
	 *            The R13 component of the matrix.
	 */
	public void setR13(double r13) {
		this.m.a13 = r13;
	}

	/**
	 * Get the R21 component of the matrix.
	 * 
	 * @return The R21 component of the matrix.
	 */
	public double getR21() {
		return this.m.a21;
	}

	/**
	 * Set the R21 component of the matrix.
	 * 
	 * @param r21
	 *            The R33 component of the matrix.
	 */
	public void setR21(double r21) {
		this.m.a21 = r21;
	}

	/**
	 * Get the R22 component of the matrix.
	 * 
	 * @return The R22 component of the matrix.
	 */
	public double getR22() {
		return this.m.a22;
	}

	/**
	 * Set the R22 component of the matrix.
	 * 
	 * @param r22
	 *            The R22 component of the matrix.
	 */
	public void setR22(double r22) {
		this.m.a22 = r22;
	}

	/**
	 * Get the R23 component of the matrix.
	 * 
	 * @return The R23 component of the matrix.
	 */
	public double getR23() {
		return this.m.a23;
	}

	/**
	 * Set the R23 component of the matrix.
	 * 
	 * @param r23
	 *            The R23 component of the matrix.
	 */
	public void setR23(double r23) {
		this.m.a23 = r23;
	}

	/**
	 * Get the R31 component of the matrix.
	 * 
	 * @return The R31 component of the matrix.
	 */
	public double getR31() {
		return this.m.a31;
	}

	/**
	 * Set the R31 component of the matrix.
	 * 
	 * @param r31
	 *            The R31 component of the matrix.
	 */
	public void setR31(double r31) {
		this.m.a31 = r31;
	}

	/**
	 * Get the R32 component of the matrix.
	 * 
	 * @return The R32 component of the matrix.
	 */
	public double getR32() {
		return this.m.a32;
	}

	/**
	 * Set the R32 component of the matrix.
	 * 
	 * @param r32
	 *            The R32 component of the matrix.
	 */
	public void setR32(double r32) {
		this.m.a32 = r32;
	}

	/**
	 * Get the R33 component of the matrix.
	 * 
	 * @return The R33 component of the matrix.
	 */
	public double getR33() {
		return this.m.a33;
	}

	/**
	 * Set the R33 component of the matrix.
	 * 
	 * @param r33
	 *            The R33 component of the matrix.
	 */
	public void setR33(double r33) {
		this.m.a33 = r33;
	}

	/**
	 * Get the rotation in radians for this matrix about the given axis.
	 * 
	 * @param axis
	 *            The axis of rotation.
	 * @return The rotation in radians.
	 */
	public double getRotationAboutAxis(RotationMatrix.RotationType axis) {
		// determine the matrix type
		if (axis == RotationType.X_ROTATION) {
			return Math.atan2(this.m.a32, this.m.a33);
		} else if (axis == RotationType.Y_ROTATION) {
			return Math.atan2(-this.m.a31, Math.sqrt(Math.pow(this.m.a32, 2)
					+ Math.pow(this.m.a33, 2)));
		} else {
			return Math.atan2(this.m.a21, this.m.a11);
		}
	}

	/**
	 * Convert this rotation matrix into a 4-dimensional vector (e.g., a
	 * Quaternion).
	 * 
	 * @return This matrix as a Quaternion (Vector4)
	 */
	public Vector4 toVector4() {
		// thanks to NASA Mission Planning & Analysis Division -- July 1977
		double t1 = Math.atan2(-this.m.a23, this.m.a33);
		double t2 = Math.atan2(this.m.a13,
				Math.sqrt(1.0 - Math.pow(this.m.a13, 2.0)));
		double t3 = Math.atan2(-this.m.a12, this.m.a11);

		// convert Euler to Quaternion
		double w = -Math.sin(t1 / 2.0) * Math.sin(t2 / 2.0)
				* Math.sin(t3 / 2.0) + Math.cos(t1 / 2.0) * Math.cos(t2 / 2.0)
				* Math.cos(t3 / 2.0);
		double x = Math.sin(t1 / 2.0) * Math.cos(t2 / 2.0) * Math.cos(t3 / 2.0)
				+ Math.sin(t2 / 2.0) * Math.sin(t3 / 2.0) * Math.cos(t1 / 2.0);
		double y = -Math.sin(t1 / 2.0) * Math.sin(t3 / 2.0)
				* Math.cos(t2 / 2.0) + Math.sin(t2 / 2.0) * Math.cos(t1 / 2.0)
				* Math.cos(t3 / 2.0);
		double z = Math.sin(t1 / 2.0) * Math.sin(t2 / 2.0) * Math.cos(t3 / 2.0)
				+ Math.sin(t3 / 2.0) * Math.cos(t2 / 2.0) * Math.cos(t2 / 2.0);		
		
		return new Vector4(w, x, y, z);
	}

	/**
	 * Check if the given Item is the same as the rotation matrix. Two
	 * rotation matrices are the same if all components of the matrix are equal.
	 * 
	 * @param o
	 *            The Item to check.
	 * @return If the Item is equal to this RotationMatrix.
	 */
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof RotationMatrix) {
			RotationMatrix r = (RotationMatrix) o;
			return this.getR11() == r.getR11() && this.getR12() == r.getR12()
					&& this.getR13() == r.getR13()
					&& this.getR21() == r.getR21()
					&& this.getR22() == r.getR22()
					&& this.getR23() == r.getR23()
					&& this.getR31() == r.getR31()
					&& this.getR32() == r.getR32()
					&& this.getR33() == r.getR33();
		} else {
			return false;
		}
	}

	/**
	 * Create a clone of this RotationMatrix.
	 * 
	 * @return A clone of this RotationMatrix.
	 */
	public RotationMatrix clone() {
		return new RotationMatrix(this.getR11(), this.getR12(), this.getR13(),
				this.getR21(), this.getR22(), this.getR23(), this.getR31(),
				this.getR32(), this.getR33());
	}
}
