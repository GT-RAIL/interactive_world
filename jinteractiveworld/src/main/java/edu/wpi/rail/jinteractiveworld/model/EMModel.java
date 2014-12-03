package edu.wpi.rail.jinteractiveworld.model;

import edu.wpi.rail.jinteractiveworld.data.DataPoint;
import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import weka.clusterers.EM;
import weka.core.Instance;
import weka.core.Instances;
import weka.filters.Filter;
import weka.filters.unsupervised.attribute.Remove;

import java.util.ArrayList;
import java.util.List;

/**
 * A EMModel is a model which is based on a EM clustering. After running
 * EM, the densest cluster is picked and its mean values are set as the best
 * location. Density is defined to be the average distance between all points in
 * the cluster.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 3, 2014
 */
public class EMModel implements Model {

	private EM em;
	private Placement best;
	private double min;
	private DataSet data;
	private String referenceFrame;
	private Item item;
	private Room room;
	private Surface surface;

	/**
	 * Create a clustering model based on the given data set. This model will be
	 * initially trained upon instantiation.
	 *
	 * @param data
	 *            The data set for the model.
	 * @param item
	 *            The item for this model.
	 * @param room
	 *            The target room for this model.
	 * @param surface
	 *            The target surface for this model.
	 * @param referenceFrame
	 *            The reference frame for this model.
	 */
	public EMModel(DataSet data, Item item, Room room, Surface surface, String referenceFrame) {
		this.data = data;
		this.item = item;
		this.room = room;
		this.surface = surface;
		this.referenceFrame = referenceFrame;
		this.train();
	}

	/**
	 * Add a data point to this model. The model will the be retrained.
	 *
	 * @param point
	 *            The data point to add to this model.
	 */
	@Override
	public void add(DataPoint point) {
		this.data.add(point);
		// retrain
		this.train();
	}

	/**
	 * Get the size of the model.
	 *
	 * @return The size of the model.
	 */
	public int size() {
		return this.data.size();
	}

	/**
	 * Get all the associated data for this model.
	 *
	 * @return The associated data for this model.
	 */
	@Override
	public List<DataPoint> getData() {
		// copy into an array list
		ArrayList<DataPoint> list = new ArrayList<DataPoint>();
		for (int i = 0; i < this.data.size(); i++) {
			list.add(this.data.get(i));
		}
		return list;
	}

	/**
	 * Get the reference frame for this model.
	 *
	 * @return The reference frame for this model.
	 */
	@Override
	public String getReferenceFrame() {
		return this.referenceFrame;
	}

	/**
	 * Get the target item for this model.
	 *
	 * @return The target item for this model.
	 */
	@Override
	public Item getItem() {
		return this.item;
	}

	/**
	 * Get the target room for this model.
	 *
	 * @return The target room for this model.
	 */
	@Override
	public Room getRoom() {
		return this.room;
	}

	/**
	 * Get the target surface for this model.
	 *
	 * @return The target surface for this model.
	 */
	@Override
	public Surface getSurface() {
		return this.surface;
	}

	/**
	 * Train the model.
	 */
	@Override
	public void train() {
		try {
			this.em = new EM();
			this.best = null;
			this.min = Double.POSITIVE_INFINITY;

			// get the instances
			Instances instances = this.data.toInstances();

			// remove z for now
			Remove rm = new Remove();
			rm.setAttributeIndicesArray(new int[] { DataSet.Z_ATTRIBUTE.index() });
			rm.setInputFormat(instances);
			Instances newInstances = Filter.useFilter(instances, rm);

			// run EM
			this.em.buildClusterer(newInstances);

			// get the results
			double clusterData[][][] = this.em.getClusterModelsNumericAtts();

			// cluster each point
			@SuppressWarnings("unchecked")
			ArrayList<Instance>[] clusters = (ArrayList<Instance>[]) new ArrayList[clusterData.length];
			for (int i = 0; i < clusters.length; i++) {
				clusters[i] = new ArrayList<Instance>();
			}
			for (int i = 0; i < newInstances.numInstances(); i++) {
				Instance inst = newInstances.instance(i);
				int clust = this.em.clusterInstance(inst);
				clusters[clust].add(inst);
			}

			// determine the densest cluster
			for (int m = 0; m < clusters.length; m++) {
				ArrayList<Instance> curInsts = clusters[m];
				double distance = 0;
				for (int i = 0; i < curInsts.size(); i++) {
					Instance instI = curInsts.get(i);
					for (int j = 0; j < curInsts.size(); j++) {
						if (i != j) {
							Instance instJ = curInsts.get(j);
							// get each attribute
							double sum = 0;
							for (int k = 0; k < instI.numAttributes(); k++) {
								sum += Math.pow(
										instI.value(k) - instJ.value(k), 2.0);
							}
							// get the distance
							distance += Math.sqrt(sum);
						}
					}
				}
				// average
				double density = distance
						/ (curInsts.size() * (curInsts.size() - 1)) * 1.0
						/ (((double) curInsts.size()) / ((double) this.size()));

				// check for a new best
				if (density < this.min) {
					this.min = density;

					double x = clusterData[m][0][0];
					double y = clusterData[m][1][0];
					double z = 0.0;
					double theta = clusterData[m][2][0];
					this.best = new Placement(this.item, this.room, this.surface, this.referenceFrame, new Point(x, y, z), theta);
				}
			}
		} catch (Exception e) {
			System.err.println("[ERROR]: Could not train model: "
					+ e.getMessage());
		}
	}

	/**
	 * Get the most likely placement location of the target item based on this
	 * model.
	 * 
	 * @return The most likely placement location of the target item based on
	 *         this model.
	 */
	@Override
	public Placement getPlacementLocation() {
		return this.best;
	}

	/**
	 * Get the value that the was the result of the placement decision. Smaller
	 * values are better in this model.
	 * 
	 * @return The value that the was the result of the placement decision.
	 */
	@Override
	public double getDecisionValue() {
		return this.min;
	}
}