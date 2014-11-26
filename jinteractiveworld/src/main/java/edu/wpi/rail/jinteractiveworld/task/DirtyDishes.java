package edu.wpi.rail.jinteractiveworld.task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.ClusteringModel;
import edu.wpi.rail.jinteractiveworld.model.Model;
import edu.wpi.rail.jinteractiveworld.model.relevance.ModelSetTable;
import edu.wpi.rail.jinteractiveworld.model.relevance.RelevanceTable;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

/**
 * The DirtyDishes Task contains information about putting away dirty dishes.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 4, 2014
 */
public class DirtyDishes extends Task {

	/**
	 * The name of the task.
	 */
	private static final String NAME = "Dirty Dishes";

	/**
	 * The name of the data folder for the data.
	 */
	private static final String DATA_LOCATION = "data/dirty_dishes";

	/**
	 * The set of associated items for this task.
	 */
	private static final Item[] ITEMS = { new Plate(), new Spoon(), new Fork(),
			new Cup() };

	/**
	 * Create a DirtyDishes task based on the given relevance table.
	 * 
	 * @param table
	 *            The relevance table for the Task.
	 * @param models
	 *            The models for the Task.
	 */
	public DirtyDishes(RelevanceTable table, Set<Model> models) {
		super(DirtyDishes.NAME, table, models, new HashSet<Item>(
				Arrays.asList(DirtyDishes.ITEMS)));
	}

	/**
	 * Run the task.
	 * 
	 * @param args
	 *            Ignored.
	 */
	public static void main(String[] args) {
		System.out.println("== Table Setting Task Execution ==");
		System.out.println();

		// load the data
		System.out.print("Loading data...");
		ArrayList<DataSet> data = DataSet
				.loadDataFolder(DirtyDishes.DATA_LOCATION);
		System.out.println(" done!");
		System.out.println(data.size() + " data sets loaded.");
		System.out.println();

		// create models for each
		System.out.println("Training models...");
		HashSet<Model> models = new HashSet<Model>();
		for (DataSet ds : data) {
			Object ref = DataSet.extractReferenceFromName(ds);
			Item targ = DataSet.extractTargetFromName(ds);

			System.out.print("\tTraining " + targ.getName() + " w.r.t. "
					+ ref.getName() + "...");
			Model m = new ClusteringModel(ds, ref, targ);
			models.add(m);
			System.out.println(" done!");
		}
		System.out.println(models.size() + " models trained.");
		System.out.println();

		// create the relevance table
		System.out.print("Creating relevance table...");
		RelevanceTable table = new ModelSetTable(models);
		System.out.println(" done!");
		System.out.println();

		System.out.println("Executing task...");
		DirtyDishes task = new DirtyDishes(table, models);
		task.execute();
		System.out.println("Task execution complete!");
	}
}
