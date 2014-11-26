package edu.wpi.rail.jinteractiveworld.world;

/**
 * The TV object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class TV extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "TV";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 2.14;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.64;

	/**
	 * Create a new TV object.
	 */
	public TV() {
		super(TV.NAME, TV.WIDTH, TV.HEIGHT);
	}
}
