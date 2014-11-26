package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Oven object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Oven extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Oven";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.215;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.66;

	/**
	 * Create a new Oven object.
	 */
	public Oven() {
		super(Oven.NAME, Oven.WIDTH, Oven.HEIGHT);
	}
}
