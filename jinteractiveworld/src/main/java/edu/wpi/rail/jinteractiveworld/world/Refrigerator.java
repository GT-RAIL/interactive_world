package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Refrigerator object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Refrigerator extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Refrigerator";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.883;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.8;

	/**
	 * Create a new Refrigerator object.
	 */
	public Refrigerator() {
		super(Refrigerator.NAME, Refrigerator.WIDTH, Refrigerator.HEIGHT);
	}
}
