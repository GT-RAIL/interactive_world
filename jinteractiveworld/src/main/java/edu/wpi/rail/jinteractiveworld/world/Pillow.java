package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Pillow object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Pillow extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Pillow";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.64;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.35;

	/**
	 * Create a new Pillow object.
	 */
	public Pillow() {
		super(Pillow.NAME, Pillow.WIDTH, Pillow.HEIGHT);
	}
}
