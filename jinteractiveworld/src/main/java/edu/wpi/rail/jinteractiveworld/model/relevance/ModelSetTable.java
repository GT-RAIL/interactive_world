package edu.wpi.rail.jinteractiveworld.model.relevance;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Set;

import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A ModelSetTable contains a relevance table based on the ordering of a set of
 * models. Small values are considered better in a model.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 24, 2014
 */
public class ModelSetTable extends GroundTruthTable {

	/**
	 * Create a table based on the given models.
	 * 
	 * @param models
	 *            The models to evaluate.
	 */
	public ModelSetTable(Set<Model> models) {
		super();

		HashMap<Item, LinkedList<Object>> table = this.getHashMap();
		// used to place everything in order
		HashMap<Item, LinkedList<Double>> valueTable = new HashMap<Item, LinkedList<Double>>();

		// go through each model
		for (Model m : models) {
			Item target = m.getTarget();
			Object reference = m.getReference();
			double decisionValue = m.getDecisionValue();

			// empty list
			if (!table.containsKey(target)) {
				// create the initial lists
				LinkedList<Object> list = new LinkedList<Object>();
				list.add(reference);
				LinkedList<Double> values = new LinkedList<Double>();
				values.add(decisionValue);

				// place in the table
				table.put(target, list);
				valueTable.put(target, values);
			} else {
				// place it in order
				LinkedList<Object> list = table.get(target);
				LinkedList<Double> values = valueTable.get(target);
				int indexToPlace = 0;
				while (indexToPlace < values.size()
						&& decisionValue > values.get(indexToPlace)) {
					indexToPlace++;
				}

				// add each
				list.add(indexToPlace, reference);
				values.add(indexToPlace, decisionValue);
			}
		}
	}
}
