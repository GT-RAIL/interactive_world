package edu.wpi.rail.jinteractiveworld.model.relevance;

import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

import java.util.HashMap;
import java.util.LinkedList;

/**
 * A HashMapTable is a relevance table with an underlying hash-map structure.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public class HashMapTable implements RelevanceTable {

	private HashMap<Item, LinkedList<Object>> table;

	/**
	 * Create an empty table.
	 */
	public HashMapTable() {
		this(new HashMap<Item, LinkedList<Object>>());
	}

	/**
	 * Get the underlying HashMap for this table.
	 * 
	 * @return The underlying HashMap for this table.
	 */
	HashMap<Item, LinkedList<Object>> getHashMap() {
		return this.table;
	}

	/**
	 * Create a table based on the given hash map.
	 */
	public HashMapTable(HashMap<Item, LinkedList<Object>> table) {
		this.table = table;
	}

	/**
	 * Get the reference object for the given target regardless of state.
	 * 
	 * @param target
	 *            The target item.
	 * @return The best reference object for the target regardless of state.
	 */
	@Override
	public Object getReference(Item target) {
		if (this.table.containsKey(target)) {
			return this.table.get(target).getFirst();
		}
		return null;
	}

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
	@Override
	public Object getReference(Item target, State s) {
		if (this.table.containsKey(target)) {
			for (Object o : this.table.get(target)) {
				if (s.getObjects().contains(o)) {
					return o;
				}
			}
		}
		return null;
	}

	/**
	 * Get a priority list of reference objects for the given target of the
	 * objects available in the given state.
	 * 
	 * @param target
	 *            The target item.
	 * @return A priority list objects for the target regardless of state.
	 */
	@Override
	public LinkedList<Object> getReferences(Item target) {
		return this.table.get(target);
	}

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
	@Override
	public LinkedList<Object> getReferences(Item target, State s) {
		if (this.table.containsKey(target)) {
			LinkedList<Object> result = new LinkedList<Object>();
			// check which ones to keep based on the state
			for (Object o : this.table.get(target)) {
				if (s.getObjects().contains(o)) {
					result.add(o);
				}
			}
			return result.size() > 0 ? result : null;
		}
		return null;
	}

}
