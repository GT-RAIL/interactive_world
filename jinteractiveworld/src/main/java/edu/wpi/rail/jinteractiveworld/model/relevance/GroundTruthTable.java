package edu.wpi.rail.jinteractiveworld.model.relevance;

import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

import java.util.HashMap;
import java.util.LinkedList;

/**
 * A GroundTruthTable contains a hard coded relevance table.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public class GroundTruthTable extends HashMapTable {

	/**
	 * Create an empty ground truth table.
	 */
	public GroundTruthTable() {
		super();
	}

	/**
	 * Add a target reference pairing for this table.
	 * 
	 * @param reference
	 *            The reference object.
	 * @param target
	 *            The target item.
	 */
	public void addReference(Object reference, Item target) {
		HashMap<Item, LinkedList<Object>> table = this.getHashMap();

		if (!table.containsKey(target)) {
			table.put(target, new LinkedList<Object>());
		}
		table.get(target).add(reference);
	}
}
