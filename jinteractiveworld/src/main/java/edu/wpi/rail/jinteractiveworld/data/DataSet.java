package edu.wpi.rail.jinteractiveworld.data;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import weka.core.Attribute;
import weka.core.FastVector;
import weka.core.Instance;
import weka.core.Instances;

/**
 * A DataSet contains a collection of transformation matrices. Rotations occur
 * along the Z axis.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 6, 2014
 */
public class DataSet {

	/**
	 * The dimensionality of the data set.
	 */
	public static final int N = 4;

	/**
	 * The x attribute for the data set.
	 */
	public static final Attribute X_ATTRIBUTE = new Attribute("x");

	/**
	 * The y attribute for the data set.
	 */
	public static final Attribute Y_ATTRIBUTE = new Attribute("y");

	/**
	 * The z attribute for the data set.
	 */
	public static final Attribute Z_ATTRIBUTE = new Attribute("z");

	/**
	 * The theta attribute for the data set.
	 */
	public static final Attribute THETA_ATTRIBUTE = new Attribute("theta");

	private ArrayList<DataPoint> data;
	private String name;

	/**
	 * Create a new, empty data set and empty name.
	 */
	public DataSet() {
		this("");
	}

	/**
	 * Create a new, empty data set with the given name.
	 * 
	 * @param name
	 *            The name of the data set.
	 */
	public DataSet(String name) {
		this.data = new ArrayList<DataPoint>();
		this.name = name;
	}

	/**
	 * Get the name of the data set.
	 * 
	 * @return The name of the data set.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Set the name of the data set.
	 * 
	 * @param name
	 *            The name of the data set.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get the size of this data set.
	 * 
	 * @return The number of points in the data set.
	 */
	public int size() {
		return this.data.size();
	}

	/**
	 * Add a data point based on the given x, y, z, and theta values.
	 * 
	 * @param x
	 *            The x value of the data point.
	 * @param y
	 *            The y value of the data point.
	 * @param z
	 *            The z value of the data point.
	 * @param theta
	 *            The theta value of the data point.
	 */
	public void add(double x, double y, double z, double theta) {
		this.add(new DataPoint(x, y, z, theta));
	}

	/**
	 * Add the given data point to the data set.
	 * 
	 * @param point
	 *            The data point to add.
	 */
	public void add(DataPoint point) {
		this.data.add(point);
	}

	/**
	 * Get the data point at the given index.
	 * 
	 * @param index
	 *            The index to get.
	 * @return The data at the given index.
	 */
	public DataPoint get(int index) {
		return this.data.get(index);
	}

	/**
	 * Get the data set as a set of Weka instances.
	 * 
	 * @return The data set as a set of Weka instances.
	 */
	public Instances toInstances() {
		// attributes for the set
		FastVector attributes = new FastVector(DataSet.N);
		attributes.addElement(DataSet.X_ATTRIBUTE);
		attributes.addElement(DataSet.Y_ATTRIBUTE);
		attributes.addElement(DataSet.Z_ATTRIBUTE);
		attributes.addElement(DataSet.THETA_ATTRIBUTE);

		Instances instances = new Instances("data", attributes, this.size());
		// add each instance
		for (int i = 0; i < this.size(); i++) {
			// get the data
			DataPoint point = this.get(i);

			Instance inst = new Instance(DataSet.N);
			inst.setDataset(instances);
			// x, y, z, theta
			inst.setValue(DataSet.X_ATTRIBUTE, point.getX());
			inst.setValue(DataSet.Y_ATTRIBUTE, point.getY());
			inst.setValue(DataSet.Z_ATTRIBUTE, point.getZ());
			inst.setValue(DataSet.THETA_ATTRIBUTE, point.getTheta());

			instances.add(inst);
		}

		return instances;
	}

	/**
	 * Get the minimum x value.
	 * 
	 * @return The minimum x value.
	 */
	public double getMinX() {
		double minX = Double.POSITIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getX() < minX) {
				minX = point.getX();
			}
		}
		return minX;
	}

	/**
	 * Get the maximum x value.
	 * 
	 * @return The maximum x value.
	 */
	public double getMaxX() {
		double maxX = Double.NEGATIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getX() > maxX) {
				maxX = point.getX();
			}
		}
		return maxX;
	}

	/**
	 * Get the minimum y value.
	 * 
	 * @return The minimum y value.
	 */
	public double getMinY() {
		double minY = Double.POSITIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getY() < minY) {
				minY = point.getY();
			}
		}
		return minY;
	}

	/**
	 * Get the maximum y value.
	 * 
	 * @return The maximum y value.
	 */
	public double getMaxY() {
		double maxY = Double.NEGATIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getY() > maxY) {
				maxY = point.getY();
			}
		}
		return maxY;
	}

	/**
	 * Get the minimum z value.
	 * 
	 * @return The minimum z value.
	 */
	public double getMinZ() {
		double minZ = Double.POSITIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getZ() < minZ) {
				minZ = point.getZ();
			}
		}
		return minZ;
	}

	/**
	 * Get the maximum z value.
	 * 
	 * @return The maximum z value.
	 */
	public double getMaxZ() {
		double maxZ = Double.NEGATIVE_INFINITY;
		for (DataPoint point : this.data) {
			if (point.getZ() > maxZ) {
				maxZ = point.getZ();
			}
		}
		return maxZ;
	}
}
