package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Chair object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public class Chair extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Chair";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.29;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.44;

	/**
	 * Create a new Chair object.
	 */
	public Chair() {
		super(Chair.NAME, Chair.WIDTH, Chair.HEIGHT);
	}
}
