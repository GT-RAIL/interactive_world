package edu.wpi.rail.jinteractiveworld.data;

import java.util.ArrayList;

import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;
import weka.core.*;

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
	private Item item;
	private Room room;
	private Surface surface;
	private String referenceFrame;
	/**
	 * Create a new, empty data set.
	 */
	public DataSet() {
		this(new Item(), new Room(), new Surface(), "");
	}

	/**
	 * Create a new, empty data set.
	 *
	 * @param item
	 *            The item for this data set.
	 * @param room
	 *            The target room for this data set.
	 * @param surface
	 *            The target surface for this data set.
	 * @param referenceFrame
	 *            The reference frame for this data set.
	 */
	public DataSet(Item item, Room room, Surface surface, String referenceFrame) {
		this.item = item;
		this.room = room;
		this.surface = surface;
		this.referenceFrame = referenceFrame;
		this.data = new ArrayList<DataPoint>();
	}

	/**
	 * Get the reference frame for this model.
	 *
	 * @return The reference frame for this model.
	 */
	public String getReferenceFrame() {
		return this.referenceFrame;
	}

	/**
	 * Get the target item for this model.
	 *
	 * @return The target item for this model.
	 */
	public Item getItem() {
		return this.item;
	}

	/**
	 * Get the target room for this model.
	 *
	 * @return The target room for this model.
	 */
	public Room getRoom() {
		return this.room;
	}

	/**
	 * Get the target surface for this model.
	 *
	 * @return The target surface for this model.
	 */
	public Surface getSurface() {
		return this.surface;
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
