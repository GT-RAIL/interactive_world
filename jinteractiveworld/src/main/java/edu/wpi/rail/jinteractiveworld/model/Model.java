package edu.wpi.rail.jinteractiveworld.model;

import java.util.List;

import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A model consists of a collection of locations and an idea of ideal placement
 * locations.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public interface Model {

	/**
	 * Add a transform to this model.
	 * 
	 * @param tf
	 *            The transform to add to this model.
	 */
	public void add(Transform tf);

	/**
	 * Get the size of the model.
	 * 
	 * @return The size of the model.
	 */
	public int size();

	/**
	 * Get all the associated data for this model.
	 * 
	 * @return The associated data for this model.
	 */
	public List<Transform> getData();

	/**
	 * Get the reference object for this model.
	 * 
	 * @return The reference object for this model.
	 */
	public Object getReference();

	/**
	 * Get the target item for this model.
	 * 
	 * @return The target item for this model.
	 */
	public Item getTarget();

	/**
	 * Train the model.
	 */
	public void train();

	/**
	 * Get the most likely placement location of the target item based on this
	 * model.
	 * 
	 * @return The most likely placement location of the target item based on
	 *         this model.
	 */
	public Transform getHighestPlacementLocation();

	/**
	 * Get the value that the was the result of the placement decision.
	 * 
	 * @return The value that the was the result of the placement decision.
	 */
	public double getDecisionValue();
}
