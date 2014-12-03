package edu.wpi.rail.jinteractiveworld.world;

/**
 * An Item represents an entity in the world, such as a table or a spoon.
 * Objects have an associated name which should be unique.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public abstract class Object {

	private String name;
	private double width;
	private double height;

	/**
	 * Create an Item with the given name and bounding box dimensions in
	 * meters.
	 * 
	 * @param name
	 *            The name of the Item.
	 * @param width
	 *            The width the Item.
	 * @param height
	 *            The height of the Item.
	 */
	public Object(String name, double width, double height) {
		this.name = name;
		this.width = width;
		this.height = height;
	}

	/**
	 * Get the name of the Item.
	 * 
	 * @return The name of the object.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Set the name of the Item.
	 * 
	 * @param name
	 *            The name of the Item.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get the width of the Item.
	 * 
	 * @return The width of the object.
	 */
	public double getWidth() {
		return this.width;
	}

	/**
	 * Set the width of the Item.
	 * 
	 * @param width
	 *            The width the Item.
	 */
	public void setWidth(double width) {
		this.width = width;
	}

	/**
	 * Get the height of the Item.
	 * 
	 * @return The height of the object.
	 */
	public double getHeight() {
		return this.height;
	}

	/**
	 * Set the height of the Item.
	 * 
	 * @param height
	 *            The height the Item.
	 */
	public void setHeight(double height) {
		this.height = height;
	}

	/**
	 * Get the hash code for this Item which is the same as the hash code for
	 * the Item's name.
	 * 
	 * @return The hash code.
	 */
	@Override
	public int hashCode() {
		return name.hashCode();
	}

	/**
	 * Check if the two Objects are the same. Two Objects are the same if they
	 * are both edu.wpi.rail.jinteractiveworld.world.Item objects and have the
	 * same name.
	 * 
	 * @param o
	 *            The Item to check.
	 * @return If the Item is equal to this
	 *         edu.wpi.rail.jinteractiveworld.world.Item.
	 */
	@Override
	public boolean equals(java.lang.Object o) {
		if (o == this) {
			return true;
		} else if (o instanceof Object) {
			return ((Object) o).getName().equals(this.name);
		} else {
			return false;
		}
	}
}
