package edu.wpi.rail.jinteractiveworld.model;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A DataSetModel is a model which is based on a DataSet. This is an abstract
 * class which does not implement the getHighestPlacementLocation method.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 18, 2014
 */
public abstract class DataSetModel implements Model {

	protected DataSet data;
	private Object reference;
	private Item target;

	/**
	 * Create a new DataSetModel. The model will be initially trained.
	 * 
	 * @param data
	 *            The data set for the model.
	 * @param reference
	 *            The reference object for this model.
	 * @param target
	 *            The target item for this model.
	 */
	public DataSetModel(DataSet data, Object reference, Item target) {
		this.data = data;
		this.reference = reference;
		this.target = target;
		this.train();
	}

	/**
	 * Add a transform to this model. The model will the be retrained.
	 * 
	 * @param tf
	 *            The transform to add to this model.
	 */
	@Override
	public void add(Transform tf) {
		this.data.add(tf);
		// retrain
		this.train();
	}

	/**
	 * Get the size of the model.
	 * 
	 * @return The size of the model.
	 */
	@Override
	public int size() {
		return this.data.size();
	}

	/**
	 * Get all the associated data for this model.
	 * 
	 * @return The associated data for this model.
	 */
	@Override
	public List<Transform> getData() {
		// copy into an array list
		ArrayList<Transform> list = new ArrayList<Transform>();
		for (int i = 0; i < this.data.size(); i++) {
			list.add(this.data.get(i));
		}
		return list;
	}

	/**
	 * Get the reference object for this model.
	 * 
	 * @return The reference object for this model.
	 */
	@Override
	public Object getReference() {
		return this.reference;
	}

	/**
	 * Get the target item for this model.
	 * 
	 * @return The target item for this model.
	 */
	@Override
	public Item getTarget() {
		return this.target;
	}
}
