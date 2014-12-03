package edu.wpi.rail.jinteractiveworld.model.transform;

import org.ejml.alg.fixed.FixedOps4;
import org.ejml.data.FixedMatrix4x4_64F;

/**
 * A Transform represents a transformation matrix, that is, a rotation and
 * translational component in 3-dimensions.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 6, 2014
 */
public class Transform {

	// internal storage
	private FixedMatrix4x4_64F m;

	/**
	 * Create a transform at (0, 0, 0) with 0 rotation.
	 */
	public Transform() {
		this(new RotationMatrix(), new Vector3());
	}

	/**
	 * Create a matrix based on the given components.
	 * 
	 * @param r
	 *            The rotation matrix for this transform.
	 * 
	 * @param v
	 *            The translational vector for this transform.
	 */
	public Transform(RotationMatrix r, Vector3 v) {
		this(r.getR11(), r.getR12(), r.getR13(), v.getX(), r.getR21(), r
				.getR22(), r.getR23(), v.getY(), r.getR31(), r.getR32(), r
				.getR33(), v.getZ(), 0, 0, 0, 1);
	}

	/**
	 * Create a Transform based on the given components.
	 * 
	 * @param r11
	 *            The R11 component of the matrix.
	 * @param r12
	 *            The R12 component of the matrix.
	 * @param r13
	 *            The R13 component of the matrix.
	 * @param r14
	 *            The R14 component of the matrix.
	 * @param r21
	 *            The R21 component of the matrix.
	 * @param r22
	 *            The R22 component of the matrix.
	 * @param r23
	 *            The R23 component of the matrix.
	 * @param r24
	 *            The R24 component of the matrix.
	 * @param r31
	 *            The R31 component of the matrix.
	 * @param r32
	 *            The R32 component of the matrix.
	 * @param r33
	 *            The R33 component of the matrix.
	 * @param r34
	 *            The R34 component of the matrix.
	 * @param r41
	 *            The R41 component of the matrix.
	 * @param r42
	 *            The R42 component of the matrix.
	 * @param r43
	 *            The R43 component of the matrix.
	 * @param r44
	 *            The R44 component of the matrix.
	 */
	public Transform(double r11, double r12, double r13, double r14,
			double r21, double r22, double r23, double r24, double r31,
			double r32, double r33, double r34, double r41, double r42,
			double r43, double r44) {
		// copy in the values
		this.m = new FixedMatrix4x4_64F(r11, r12, r13, r14, r21, r22, r23, r24,
				r31, r32, r33, r34, r41, r42, r43, r44);
	}

	/**
	 * Set the rotational component of this transform.
	 * 
	 * @param r
	 *            The new rotation matrix to use.
	 */
	public void setRotation(RotationMatrix r) {
		this.m.a11 = r.getR11();
		this.m.a12 = r.getR12();
		this.m.a13 = r.getR13();
		this.m.a21 = r.getR21();
		this.m.a22 = r.getR22();
		this.m.a23 = r.getR23();
		this.m.a31 = r.getR31();
		this.m.a32 = r.getR32();
		this.m.a33 = r.getR33();
	}

	/**
	 * Get the rotation matrix associated with this transform.
	 * 
	 * @return The rotation matrix associated with this transform.
	 */
	public RotationMatrix getRotationMatrix() {
		return new RotationMatrix(this.m.a11, this.m.a12, this.m.a13,
				this.m.a21, this.m.a22, this.m.a23, this.m.a31, this.m.a32,
				this.m.a33);
	}

	/**
	 * Set the translational component of this transform.
	 * 
	 * @param v
	 *            The new translation vector to use.
	 */
	public void setTranslation(Vector3 v) {
		this.m.a14 = v.getX();
		this.m.a24 = v.getY();
		this.m.a34 = v.getZ();
	}

	/**
	 * Get the translational vector associated with this transform.
	 * 
	 * @return The translational vector associated with this transform.
	 */
	public Vector3 getVector3() {
		return new Vector3(this.m.a14, this.m.a24, this.m.a34);
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
	 * Get the R14 component of the matrix.
	 * 
	 * @return The R14 component of the matrix.
	 */
	public double getR14() {
		return this.m.a14;
	}

	/**
	 * Set the R14 component of the matrix.
	 * 
	 * @param r14
	 *            The R14 component of the matrix.
	 */
	public void setR14(double r14) {
		this.m.a14 = r14;
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
	 * Get the R24 component of the matrix.
	 * 
	 * @return The R24 component of the matrix.
	 */
	public double getR24() {
		return this.m.a24;
	}

	/**
	 * Set the R24 component of the matrix.
	 * 
	 * @param r24
	 *            The R24 component of the matrix.
	 */
	public void setR24(double r24) {
		this.m.a24 = r24;
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
	 * Get the R34 component of the matrix.
	 * 
	 * @return The R34 component of the matrix.
	 */
	public double getR34() {
		return this.m.a34;
	}

	/**
	 * Set the R34 component of the matrix.
	 * 
	 * @param r34
	 *            The R34 component of the matrix.
	 */
	public void setR34(double r34) {
		this.m.a34 = r34;
	}

	/**
	 * Get the R41 component of the matrix.
	 * 
	 * @return The R41 component of the matrix.
	 */
	public double getR41() {
		return this.m.a41;
	}

	/**
	 * Set the R41 component of the matrix.
	 * 
	 * @param r41
	 *            The R41 component of the matrix.
	 */
	public void setR41(double r41) {
		this.m.a41 = r41;
	}

	/**
	 * Get the R42 component of the matrix.
	 * 
	 * @return The R42 component of the matrix.
	 */
	public double getR42() {
		return this.m.a42;
	}

	/**
	 * Set the R42 component of the matrix.
	 * 
	 * @param r42
	 *            The R42 component of the matrix.
	 */
	public void setR42(double r42) {
		this.m.a42 = r42;
	}

	/**
	 * Get the R43 component of the matrix.
	 * 
	 * @return The R43 component of the matrix.
	 */
	public double getR43() {
		return this.m.a43;
	}

	/**
	 * Set the R43 component of the matrix.
	 * 
	 * @param r43
	 *            The R43 component of the matrix.
	 */
	public void setR43(double r43) {
		this.m.a43 = r43;
	}

	/**
	 * Get the R44 component of the matrix.
	 * 
	 * @return The R44 component of the matrix.
	 */
	public double getR44() {
		return this.m.a44;
	}

	/**
	 * Set the R44 component of the matrix.
	 * 
	 * @param r44
	 *            The R44 component of the matrix.
	 */
	public void setR44(double r44) {
		this.m.a44 = r44;
	}

	/**
	 * Get the inverse of this Transform.
	 * 
	 * @return The inverse of this Transform, or null if one does not exist.
	 */
	public Transform getInverse() {
		FixedMatrix4x4_64F result = new FixedMatrix4x4_64F();
		if (FixedOps4.det(this.m) != 0) {
			FixedOps4.invert(this.m, result);
			return new Transform(result.a11, result.a12, result.a13,
					result.a14, result.a21, result.a22, result.a23, result.a24,
					result.a31, result.a32, result.a33, result.a34, result.a41,
					result.a42, result.a43, result.a44);
		} else {
			return null;
		}
	}

	/**
	 * Multiply this transform by the given. That is, the resulting matrix
	 * returned C will be the following:
	 * 
	 * C = this * tf
	 * 
	 * @param tf
	 *            The transform to perform the multiplication with.
	 * @return The resulting transform.
	 */
	public Transform multiply(Transform tf) {
		FixedMatrix4x4_64F result = new FixedMatrix4x4_64F();
		FixedMatrix4x4_64F toMult = new FixedMatrix4x4_64F(tf.getR11(),
				tf.getR12(), tf.getR13(), tf.getR14(), tf.getR21(),
				tf.getR22(), tf.getR23(), tf.getR24(), tf.getR31(),
				tf.getR32(), tf.getR33(), tf.getR34(), tf.getR41(),
				tf.getR42(), tf.getR43(), tf.getR44());
		FixedOps4.mult(this.m, toMult, result);
		return new Transform(result.a11, result.a12, result.a13, result.a14,
				result.a21, result.a22, result.a23, result.a24, result.a31,
				result.a32, result.a33, result.a34, result.a41, result.a42,
				result.a43, result.a44);
	}

	/**
	 * Check if the given Item is the same as the transform. Two transforms
	 * are the same if their rotation and translational components are the same.
	 * 
	 * @param o
	 *            The Item to check.
	 * @return If the Item is equal to this Transform.
	 */
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof Transform) {
			Transform tf = (Transform) o;
			RotationMatrix r = getRotationMatrix();
			Vector3 v = getVector3();
			return r.equals(tf.getRotationMatrix())
					&& v.equals(tf.getVector3());
		} else {
			return false;
		}
	}

	/**
	 * Create a clone of this Transform.
	 * 
	 * @return A clone of this Transform.
	 */
	public Transform clone() {
		return new Transform(this.getR11(), this.getR12(), this.getR13(),
				this.getR14(), this.getR21(), this.getR22(), this.getR23(),
				this.getR24(), this.getR31(), this.getR32(), this.getR33(),
				this.getR34(), this.getR41(), this.getR42(), this.getR43(),
				this.getR44());
	}
}
