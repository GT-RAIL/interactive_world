package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Nightstand object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Nightstand extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Nightstand";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.875;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.62;

	/**
	 * Create a new Nightstand object.
	 */
	public Nightstand() {
		super(Nightstand.NAME, Nightstand.WIDTH, Nightstand.HEIGHT);
	}
}
