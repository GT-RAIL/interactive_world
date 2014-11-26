package edu.wpi.rail.jinteractiveworld.world;

/**
 * The dining table object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 21, 2014
 */
public class DiningTable extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Dining Table with Chairs";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 2.39;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 1.19;

	/**
	 * Create a new DiningTable object.
	 */
	public DiningTable() {
		super(DiningTable.NAME, DiningTable.WIDTH, DiningTable.HEIGHT);
	}
}
