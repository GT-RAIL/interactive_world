package edu.wpi.rail.jinteractiveworld.data;

import static org.junit.Assert.*;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import org.junit.Test;

import weka.core.Instance;

public class TestDataSet {

	@Test
	public void testConstructor() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());
	}

	@Test
	public void testAdd() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());

		double x1 = 0.5;
		double y1 = -0.5;
		double z1 = 5.5;
		double theta1 = Math.PI / 2.0;
		ds.add(x1, y1, z1, theta1);
		assertEquals(1, ds.size());
		assertEquals(new DataPoint(x1, y1, z1, theta1), ds.get(0));
		double x2 = 0.25;
		double y2 = -0.25;
		double z2 = -5.5;
		double theta2 = Math.PI / 4.0;
		ds.add(x2, y2, z2, theta2);
		assertEquals(2, ds.size());
		assertEquals(new DataPoint(x2, y2, z2, theta2), ds.get(1));
	}

	@Test
	public void testGetInstances() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());

		double x1 = 0.5;
		double y1 = -0.5;
		double z1 = 5.5;
		double theta1 = Math.PI / 2.0;
		ds.add(x1, y1, z1, theta1);
		assertEquals(1, ds.size());
		assertEquals(DataSet.N, ds.toInstances().numAttributes());
		assertEquals(1, ds.toInstances().numInstances());
		Instance inst1 = ds.toInstances().instance(0);
		assertEquals(DataSet.N, inst1.numAttributes());
		assertEquals(x1, inst1.value(DataSet.X_ATTRIBUTE));
		assertEquals(y1, inst1.value(DataSet.Y_ATTRIBUTE));
		assertEquals(z1, inst1.value(DataSet.Z_ATTRIBUTE));
		assertEquals(theta1, inst1.value(DataSet.THETA_ATTRIBUTE));

		double x2 = 0.25;
		double y2 = -0.25;
		double z2 = -5.5;
		double theta2 = Math.PI / 4.0;
		ds.add(x2, y2, z2, theta2);
		assertEquals(2, ds.size());
		assertEquals(DataSet.N, ds.toInstances().numAttributes());
		assertEquals(2, ds.toInstances().numInstances());
		Instance inst2 = ds.toInstances().instance(1);
		assertEquals(DataSet.N, inst2.numAttributes());
		assertEquals(x2, inst2.value(DataSet.X_ATTRIBUTE));
		assertEquals(y2, inst2.value(DataSet.Y_ATTRIBUTE));
		assertEquals(z2, inst2.value(DataSet.Z_ATTRIBUTE));
		assertEquals(theta2, inst2.value(DataSet.THETA_ATTRIBUTE));
	}

	@Test
	public void testGetMinX() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinX(), 0);

		double x1 = 0.5;
		ds.add(x1, 0, 0, 0);
		assertEquals(x1, ds.getMinX(), 0);
		double x2 = 1.5;
		ds.add(x2, 0, 0, 0);
		assertEquals(x1, ds.getMinX(), 0);
		double x3 = -1.5;
		ds.add(x3, 0, 0, 0);
		assertEquals(x3, ds.getMinX(), 0);
	}

	@Test
	public void testGetMaxX() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxX(), 0);

		double x1 = 0.5;
		ds.add(x1, 0, 0, 0);
		assertEquals(x1, ds.getMaxX(), 0);
		double x2 = 1.5;
		ds.add(x2, 0, 0, 0);
		assertEquals(x2, ds.getMaxX(), 0);
		double x3 = -1.5;
		ds.add(x3, 0, 0, 0);
		assertEquals(x2, ds.getMaxX(), 0);
	}

	@Test
	public void testGetMinY() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinY(), 0);

		double y1 = 0.5;
		ds.add(0, y1, 0, 0);
		assertEquals(y1, ds.getMinY(), 0);
		double y2 = 1.5;
		ds.add(0, y2, 0, 0);
		assertEquals(y1, ds.getMinY(), 0);
		double y3 = -1.5;
		ds.add(0, y3, 0, 0);
		assertEquals(y3, ds.getMinY(), 0);
	}

	@Test
	public void testGetMaxY() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxY(), 0);

		double y1 = 0.5;
		ds.add(0, y1, 0, 0);
		assertEquals(y1, ds.getMaxY(), 0);
		double y2 = 1.5;
		ds.add(0, y2, 0, 0);
		assertEquals(y2, ds.getMaxY(), 0);
		double y3 = -1.5;
		ds.add(0, y3, 0, 0);
		assertEquals(y2, ds.getMaxY(), 0);
	}

	@Test
	public void testGetMinZ() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinZ(), 0);

		double z1 = 0.5;
		ds.add(0, 0, z1, 0);
		assertEquals(z1, ds.getMinZ(), 0);
		double z2 = 1.5;
		ds.add(0, 0, z2, 0);
		assertEquals(z1, ds.getMinZ(), 0);
		double z3 = -1.5;
		ds.add(0, 0, z3, 0);
		assertEquals(z3, ds.getMinZ(), 0);
	}

	@Test
	public void testGetMaxZ() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxZ(), 0);

		double z1 = 0.5;
		ds.add(0, 0, z1, 0);
		assertEquals(z1, ds.getMaxZ(), 0);
		double z2 = 1.5;
		ds.add(0, 0, z2, 0);
		assertEquals(z2, ds.getMaxZ(), 0);
		double z3 = -1.5;
		ds.add(0, 0, z3, 0);
		assertEquals(z2, ds.getMaxZ(), 0);
	}
}
