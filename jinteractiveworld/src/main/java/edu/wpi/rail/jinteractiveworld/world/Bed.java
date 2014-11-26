package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Bed object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Bed extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Bed";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.7;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 2.15;

	/**
	 * Create a new Bed object.
	 */
	public Bed() {
		super(Bed.NAME, Bed.WIDTH, Bed.HEIGHT);
	}
}
