package edu.wpi.rail.jinteractiveworld.task;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.relevance.RelevanceTable;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.TransformTree;
import edu.wpi.rail.jinteractiveworld.visualization.model.StateFrame;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;

/**
 * A Task represents a complete task in the world. Tasks have an associated list
 * of items that are relevant to the task as well as a relevance table which
 * specifies which models and frames are important during decision making.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 4, 2014
 */
public abstract class Task {

	private String name;
	private Set<Item> associatedItems;
	private RelevanceTable table;
	private Set<Model> models;

	/**
	 * Create a new Task with the given name and relevance table. An empty set
	 * of associated items will be created.
	 * 
	 * @param name
	 *            The name of the Task.
	 * @param models
	 *            The models for the Task.
	 * @param table
	 *            The relevance table used for decision making.
	 */
	public Task(String name, RelevanceTable table, Set<Model> models) {
		this.name = name;
		this.table = table;
		this.associatedItems = new HashSet<Item>();
		this.models = models;
	}

	/**
	 * Create a new Task with the given name, relevance table, and set of
	 * associated items.
	 * 
	 * @param name
	 *            The name of the Task.
	 * @param table
	 *            The relevance table used for decision making.
	 * @param models
	 *            The models for the Task.
	 * @param items
	 *            The associated items for this Task.
	 */
	public Task(String name, RelevanceTable table, Set<Model> models,
			Set<Item> items) {
		this(name, table, models);
		this.addAssociatedItems(items);
	}

	/**
	 * Get the name of the Task.
	 * 
	 * @return The name of the Task.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Set the name of the Task.
	 * 
	 * @param name
	 *            The name of the Task.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get the RelevanceTable associated with this Task.
	 * 
	 * @return The RelevanceTable associated with this Task.
	 */
	public RelevanceTable getRelevanceTable() {
		return this.table;
	}

	/**
	 * Set the RelevanceTable associated with this Task.
	 * 
	 * @param table
	 *            The RelevanceTable associated with this Task.
	 */
	public void setRelevanceTable(RelevanceTable table) {
		this.table = table;
	}

	/**
	 * Get the set of associated items for this Task.
	 * 
	 * @return The set of associated items for this Task.
	 */
	public Set<Item> getAssociatedItems() {
		return this.associatedItems;
	}

	/**
	 * Get the set of models for this Task.
	 * 
	 * @return The set of models for this Task.
	 */
	public Set<Model> getModels() {
		return this.models;
	}

	/**
	 * Add an associated item to this Task.
	 * 
	 * @param item
	 *            The item to add.
	 */
	public void addAssociatedItem(Item item) {
		this.associatedItems.add(item);
	}

	/**
	 * Add a set of associated items to this Task.
	 * 
	 * @param item
	 *            The set of items to add.
	 */
	public void addAssociatedItems(Set<Item> items) {
		this.associatedItems.addAll(items);
	}

	/**
	 * Execute the task.
	 */
	public void execute() {
		// grab the table and the models
		RelevanceTable rel = this.getRelevanceTable();
		Set<Model> models = this.getModels();
		HashSet<Item> items = new HashSet<Item>(this.getAssociatedItems());

		// initialize the state
		State state = new State();

		// create a TF tree
		TransformTree tfTree = new TransformTree();

		// add the best reference as the initial item
		Object initRef = null;
		double initMin = Double.POSITIVE_INFINITY;
		for (Model m : models) {
			System.out
					.println(m.getTarget().getName() + " w.r.t. "
							+ m.getReference().getName() + " = "
							+ m.getDecisionValue());
			if (m.getDecisionValue() < initMin /*&& !(m.getReference() instanceof Nightstand)&& !(m.getReference() instanceof CoffeeTable)*/) {
				initRef = m.getReference();
				initMin = m.getDecisionValue();
			}
		}
		String initFrame = initRef.getName();
		System.out.println("Setting reference frame: " + initFrame);
		tfTree.addFrame(TransformTree.GLOBAL_FRAME, initFrame, new Transform());
		state.add(initRef,
				tfTree.getTransform(TransformTree.GLOBAL_FRAME, initFrame));

		// search for the next best item to place
		int leftToPlace = items.size();
		while (leftToPlace > 0) {
			double min = Double.POSITIVE_INFINITY;
			Item target = null;
			for (Model m : models) {
				double value = m.getDecisionValue();
				Item curTarget = m.getTarget();

				// check if we can even place this
				if (value < min && items.contains(curTarget)) {
					min = value;
					target = curTarget;
				}
			}

			// now place the current item
			Object reference = rel.getReference(target, state);
			// check where the place goes relative to this
			for (Model m : models) {
				if (m.getReference().equals(reference)
						&& m.getTarget().equals(target)) {
					// get the placement
					Transform tf = m.getHighestPlacementLocation();
					String referenceFrame = reference.getName();
					String targetFrame = target.getName();
					System.out.println("Placing "
							+ targetFrame
							+ " w.r.t. "
							+ referenceFrame
							+ " at ("
							+ tf.getR14()
							+ ", "
							+ tf.getR24()
							+ ") "
							+ tf.getRotationMatrix().getRotationAboutAxis(
									RotationMatrix.RotationType.Z_ROTATION)
							+ " rads");
					tfTree.addFrame(referenceFrame, targetFrame, tf);
					// add it to the state
					state.add(target, tfTree.getTransform(
							TransformTree.GLOBAL_FRAME, targetFrame));
				}
			}

			// update the count
			items.remove(target);
			leftToPlace = items.size();
		}

		// visualize it
		StateFrame sf = new StateFrame(state);
		sf.setVisible(true);
	}
}
