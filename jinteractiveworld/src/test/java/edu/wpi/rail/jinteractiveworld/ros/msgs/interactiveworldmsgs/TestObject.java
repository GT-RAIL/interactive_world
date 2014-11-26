package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import static org.junit.Assert.*;

import javax.json.Json;
import javax.json.JsonObject;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.world.Bed;
import edu.wpi.rail.jrosbridge.messages.Message;

public class TestObject {

	private Object empty, o1, o2;

	@Before
	public void setUp() {
		empty = new Object();
		o1 = new Object("name", 1.5, 3.0);
		o2 = new Object(new Bed());
	}

	@Test
	public void testConstructor() {
		assertEquals("", empty.getName());
		assertEquals(0.0, empty.getWidth(), 0);
		assertEquals(0.0, empty.getHeight(), 0);

		assertEquals("{\"name\":\"\",\"width\":0.0,\"height\":0.0}",
				empty.toString());

		assertEquals(3, empty.toJsonObject().size());
		assertEquals("", empty.toJsonObject().getString(Object.FIELD_NAME));
		assertEquals(0.0, empty.toJsonObject().getJsonNumber(Object.FIELD_WIDTH)
				.doubleValue(), 0);
		assertEquals(0.0, empty.toJsonObject().getJsonNumber(Object.FIELD_HEIGHT)
				.doubleValue(), 0);

		assertEquals(Object.TYPE, empty.getMessageType());
	}

	@Test
	public void testStringDoubleAndDoubleConstructor() {
		assertEquals("name", o1.getName());
		assertEquals(1.5, o1.getWidth(), 0);
		assertEquals(3.0, o1.getHeight(), 0);

		assertEquals("{\"name\":\"name\",\"width\":1.5,\"height\":3.0}",
				o1.toString());

		assertEquals(3, o1.toJsonObject().size());
		assertEquals("name", o1.toJsonObject().getString(Object.FIELD_NAME));
		assertEquals(1.5, o1.toJsonObject().getJsonNumber(Object.FIELD_WIDTH)
				.doubleValue(), 0);
		assertEquals(3.0, o1.toJsonObject().getJsonNumber(Object.FIELD_HEIGHT)
				.doubleValue(), 0);

		assertEquals(Object.TYPE, o1.getMessageType());
	}

	@Test
	public void testObjectConstructor() {
		Bed b = new Bed();
		assertEquals(b.getName(), o2.getName());
		assertEquals(b.getWidth(), o2.getWidth(), 0);
		assertEquals(b.getHeight(), o2.getHeight(), 0);

		assertEquals("{\"name\":\"Bed\",\"width\":1.7,\"height\":2.15}",
				o2.toString());

		assertEquals(3, o2.toJsonObject().size());
		assertEquals(b.getName(), o2.toJsonObject()
				.getString(Object.FIELD_NAME));
		assertEquals(b.getWidth(),
				o2.toJsonObject().getJsonNumber(Object.FIELD_WIDTH)
						.doubleValue(), 0);
		assertEquals(b.getHeight(),
				o2.toJsonObject().getJsonNumber(Object.FIELD_HEIGHT)
						.doubleValue(), 0);

		assertEquals(Object.TYPE, o2.getMessageType());
	}

	@Test
	public void testSetMessageType() {
		empty.setMessageType("test");
		assertEquals("test", empty.getMessageType());
	}

	@Test
	public void testHashCode() {
		assertEquals(empty.toString().hashCode(), empty.hashCode());
		assertEquals(o1.toString().hashCode(), o1.hashCode());
		assertEquals(o2.toString().hashCode(), o2.hashCode());
	}

	@Test
	public void testEquals() {
		assertFalse(empty.equals(o1));
		assertFalse(empty.equals(o2));
		assertFalse(o1.equals(empty));
		assertFalse(o1.equals(o2));
		assertFalse(o2.equals(o1));
		assertFalse(o2.equals(empty));

		assertTrue(o1.equals(o1));
		assertTrue(o2.equals(o2));
		assertTrue(empty.equals(empty));
	}

	@Test
	public void testEqualsWrongObject() {
		assertFalse(o1.equals(new String(o1.toString())));
	}

	@Test
	public void testClone() {
		Object clone = o1.clone();
		assertEquals(o1.toString(), clone.toString());
		assertEquals(o1.toJsonObject(), clone.toJsonObject());
		assertEquals(o1.getMessageType(), clone.getMessageType());
		assertEquals(o1.getName(), clone.getName());
		assertEquals(o1.getWidth(), clone.getWidth(), 0);
		assertEquals(o1.getHeight(), clone.getHeight(), 0);
		assertNotSame(o1, clone);
		assertNotSame(o1.toString(), clone.toString());
		assertNotSame(o1.toJsonObject(), clone.toJsonObject());
	}

	@Test
	public void testFromJsonString() {
		Object p = Object.fromJsonString(o1.toString());
		assertEquals(o1.toString(), p.toString());
		assertEquals(o1.toJsonObject(), p.toJsonObject());
		assertEquals(o1.getMessageType(), p.getMessageType());
		assertEquals(o1.getName(), p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
		assertNotSame(o1, p);
		assertNotSame(o1.toString(), p.toString());
		assertNotSame(o1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromMessage() {
		Message m = new Message(o1.toString());
		Object p = Object.fromMessage(m);
		assertEquals(o1.toString(), p.toString());
		assertEquals(o1.toJsonObject(), p.toJsonObject());
		assertEquals(o1.getMessageType(), p.getMessageType());
		assertEquals(o1.getName(), p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
		assertNotSame(o1, p);
		assertNotSame(o1.toString(), p.toString());
		assertNotSame(o1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromJsonObject() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Object.FIELD_NAME, o1.getName())
				.add(Object.FIELD_WIDTH, o1.getWidth())
				.add(Object.FIELD_HEIGHT, o1.getHeight()).build();
		Object p = Object.fromJsonObject(jsonObject);
		assertEquals(o1.toString(), p.toString());
		assertEquals(o1.toJsonObject(), p.toJsonObject());
		assertEquals(o1.getMessageType(), p.getMessageType());
		assertEquals(o1.getName(), p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
		assertNotSame(o1, p);
		assertNotSame(o1.toString(), p.toString());
		assertNotSame(o1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromJsonObjectNoName() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Object.FIELD_WIDTH, o1.getWidth())
				.add(Object.FIELD_HEIGHT, o1.getHeight()).build();
		Object p = Object.fromJsonObject(jsonObject);
		assertEquals("", p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
	}

	@Test
	public void testFromJsonObjectNoWidth() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Object.FIELD_NAME, o1.getName())
				.add(Object.FIELD_HEIGHT, o1.getHeight()).build();
		Object p = Object.fromJsonObject(jsonObject);
		assertEquals(o1.getName(), p.getName());
		assertEquals(0.0, p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
	}

	@Test
	public void testFromJsonObjectNoHeight() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Object.FIELD_NAME, o1.getName())
				.add(Object.FIELD_WIDTH, o1.getWidth()).build();
		Object p = Object.fromJsonObject(jsonObject);
		assertEquals(o1.getName(), p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(0.0, p.getHeight(), 0);
	}
}
