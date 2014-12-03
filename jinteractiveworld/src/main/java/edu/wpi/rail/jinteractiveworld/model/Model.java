package edu.wpi.rail.jinteractiveworld.model;

import java.util.List;
import edu.wpi.rail.jinteractiveworld.data.DataPoint;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;

/**
 * A model consists of a collection of locations and an idea of ideal placement
 * locations.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public interface Model {

	/**
	 * Add a data point to this model.
	 * 
	 * @param point
	 *            The transform to add to this model.
	 */
	public void add(DataPoint point);

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
	public List<DataPoint> getData();

	/**
	 * Get the reference frame for this model.
	 * 
	 * @return The reference frame for this model.
	 */
	public String getReferenceFrame();

	/**
	 * Get the target item for this model.
	 *
	 * @return The target item for this model.
	 */
	public Item getItem();

	/**
	 * Get the target room for this model.
	 *
	 * @return The target room for this model.
	 */
	public Room getRoom();

	/**
	 * Get the target surface for this model.
	 *
	 * @return The target surface for this model.
	 */
	public Surface getSurface();

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
	public Placement getPlacementLocation();

	/**
	 * Get the value that the was the result of the placement decision.
	 * 
	 * @return The value that the was the result of the placement decision.
	 */
	public double getDecisionValue();
}
