package edu.wpi.rail.jinteractiveworld.world;

/**
 * The CoffeeTable object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class CoffeeTable extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Coffee Table";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.54;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.84;

	/**
	 * Create a new CoffeeTable object.
	 */
	public CoffeeTable() {
		super(CoffeeTable.NAME, CoffeeTable.WIDTH, CoffeeTable.HEIGHT);
	}
}
