package edu.wpi.rail.jinteractiveworld.model;

import edu.wpi.rail.jinteractiveworld.data.*;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import weka.clusterers.EM;
import weka.core.*;
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

	public enum RankingFunction {
		CUSTOM
	}

	private EM em;
	private Placement best;
	private double decisionValue, sigmaX, sigmaY, sigmaZ, sigmaTheta;
	private DataSet data;
	private RankingFunction rankingType;

	/**
	 * Create a clustering model based on the given data set. This model will be
	 * initially trained upon instantiation.
	 *
	 * @param data
	 *            The data set for the model.
	 */
	public EMModel(DataSet data, RankingFunction rankingType) {
		this.data = data;
		this.rankingType = rankingType;
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
		return this.data.getReferenceFrame();
	}

	/**
	 * Get the target item for this model.
	 *
	 * @return The target item for this model.
	 */
	@Override
	public Item getItem() {
		return this.data.getItem();
	}

	/**
	 * Get the target room for this model.
	 *
	 * @return The target room for this model.
	 */
	@Override
	public Room getRoom() {
		return this.data.getRoom();
	}

	/**
	 * Get the target surface for this model.
	 *
	 * @return The target surface for this model.
	 */
	@Override
	public Surface getSurface() {
		return this.data.getSurface();
	}

	/**
	 * Train the model.
	 */
	@Override
	public void train() {
		try {
			this.em = new EM();
			this.best = null;
			this.decisionValue = Double.POSITIVE_INFINITY;
			this.sigmaX = 0;
			this.sigmaY = 0;
			this.sigmaZ = 0;
			this.sigmaTheta = 0;

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

			// rank the clusters
			for (int m = 0; m < clusters.length; m++) {
				double ranking = this.determineRanking(clusters[m]);

				// check for a new best
				if (ranking < this.decisionValue) {
					this.decisionValue = ranking;
					double x = clusterData[m][0][0];
					this.sigmaX = clusterData[m][0][1];
					double y = clusterData[m][1][0];
					this.sigmaY = clusterData[m][1][1];
					double z = 0.0;
					this.sigmaZ = 0.0;
					double theta = clusterData[m][2][0];
					this.sigmaTheta = clusterData[m][2][1];
					this.best = new Placement(this.getItem(), this.getRoom(), this.getSurface(), this.getReferenceFrame(), new Point(x, y, z), theta);
				}
			}
		} catch (Exception e) {
			System.err.println("[ERROR]: Could not train model: "
					+ e.getMessage());
		}
	}

	private double determineRanking(ArrayList<Instance> instances) {
		// check the type
		double value = Double.POSITIVE_INFINITY;

		if (this.rankingType.equals(RankingFunction.CUSTOM)) {
			double distance = 0;
			for (int i = 0; i < instances.size(); i++) {
				Instance instI = instances.get(i);
				for (int j = 0; j < instances.size(); j++) {
					if (i != j) {
						Instance instJ = instances.get(j);
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
			value = distance
					/ (instances.size() * (instances.size() - 1)) * 1.0
					/ (((double) instances.size()) / ((double) this.size()));
		}

		return value;
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
		return this.decisionValue;
	}

	/**
	 * Get the standard deviation of the X.
	 *
	 * @return The standard deviation of the X.
	 */
	@Override
	public double getSigmaX() {
		return this.sigmaX;
	}

	/**
	 * Get the standard deviation of the Y.
	 *
	 * @return The standard deviation of the Y.
	 */
	@Override
	public double getSigmaY() {
		return this.sigmaY;
	}

	/**
	 * Get the standard deviation of the Z.
	 *
	 * @return The standard deviation of the Z.
	 */
	@Override
	public double getSigmaZ() {
		return this.sigmaZ;
	}

	/**
	 * Get the standard deviation of the theta.
	 *
	 * @return The standard deviation of the theta.
	 */
	@Override
	public double getSigmaTheta() {
		return this.sigmaTheta;
	}
}
