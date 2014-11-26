package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Cabinet object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Cabinet extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Cabinet";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.91;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.557;

	/**
	 * Create a new Cabinet object.
	 */
	public Cabinet() {
		super(Cabinet.NAME, Cabinet.WIDTH, Cabinet.HEIGHT);
	}
}
