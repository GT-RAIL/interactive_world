package edu.wpi.rail.jinteractiveworld.model.relevance;

import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

import java.util.LinkedList;

/**
 * A RelevanceTable gives information on ideal reference objects when placing a
 * target object.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 12, 2014
 */
public interface RelevanceTable {

	/**
	 * Get the reference object for the given target regardless of state.
	 * 
	 * @param target
	 *            The target item.
	 * @return The best reference object for the target regardless of state.
	 */
	public Object getReference(Item target);

	/**
	 * Get the reference object for the given target of the objects available in
	 * the given state.
	 * 
	 * @param target
	 *            The target item.
	 * @param s
	 *            The current state of the world.
	 * @return The best reference object for the target based on the given
	 *         state.
	 */
	public Object getReference(Item target, State s);

	/**
	 * Get a priority list of reference objects for the given target of the
	 * objects available in the given state.
	 * 
	 * @param target
	 *            The target item.
	 * @return A priority list objects for the target regardless of state.
	 */
	public LinkedList<Object> getReferences(Item target);

	/**
	 * Get a priority list of reference objects for the given target of the
	 * objects available in the given state.
	 * 
	 * @param target
	 *            The target item.
	 * @param s
	 *            The current state of the world.
	 * @return A priority list of reference object for the target based on the
	 *         given state.
	 */
	public LinkedList<Object> getReferences(Item target, State s);
}
