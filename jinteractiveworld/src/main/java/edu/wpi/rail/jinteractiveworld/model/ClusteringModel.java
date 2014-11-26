package edu.wpi.rail.jinteractiveworld.model;

import java.util.ArrayList;

import weka.clusterers.EM;
import weka.core.Instance;
import weka.core.Instances;
import weka.filters.Filter;
import weka.filters.unsupervised.attribute.Remove;
import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A ClusteringModel is a model which is based on a EM clustering. After running
 * EM, the densest cluster is picked and its mean values are set as the best
 * location. Density is defined to be the average distance between all points in
 * the cluster.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public class ClusteringModel extends DataSetModel {

	private EM em;
	private Transform best;
	private double min;

	/**
	 * Create a clustering model based on the given data set. This model will be
	 * initially trained upon instantiation.
	 * 
	 * @param data
	 *            The data set for the model.
	 * @param reference
	 *            The reference object for this model.
	 * @param target
	 *            The target item for this model.
	 */
	public ClusteringModel(DataSet data, Object reference, Item target) {
		// super calls train which initializes em and the best location
		super(data, reference, target);
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
					double theta = clusterData[m][2][0];

					Vector3 v = new Vector3(x, y);
					RotationMatrix r = new RotationMatrix(theta,
							RotationMatrix.RotationType.Z_ROTATION);
					this.best = new Transform(r, v);
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
	public Transform getHighestPlacementLocation() {
		return best;
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
