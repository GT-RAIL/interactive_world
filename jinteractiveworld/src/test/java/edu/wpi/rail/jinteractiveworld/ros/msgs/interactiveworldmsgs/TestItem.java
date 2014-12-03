package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import static org.junit.Assert.*;

import javax.json.Json;
import javax.json.JsonObject;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.rail.jrosbridge.messages.Message;

public class TestItem {

	private Item empty, o1;

	@Before
	public void setUp() {
		empty = new Item();
		o1 = new Item("name", 1.5, 3.0);
	}

	@Test
	public void testConstructor() {
		assertEquals("", empty.getName());
		assertEquals(0.0, empty.getWidth(), 0);
		assertEquals(0.0, empty.getHeight(), 0);

		assertEquals("{\"name\":\"\",\"width\":0.0,\"height\":0.0}",
				empty.toString());

		assertEquals(3, empty.toJsonObject().size());
		assertEquals("", empty.toJsonObject().getString(Item.FIELD_NAME));
		assertEquals(0.0, empty.toJsonObject().getJsonNumber(Item.FIELD_WIDTH)
				.doubleValue(), 0);
		assertEquals(0.0, empty.toJsonObject().getJsonNumber(Item.FIELD_HEIGHT)
				.doubleValue(), 0);

		assertEquals(Item.TYPE, empty.getMessageType());
	}

	@Test
	public void testStringDoubleAndDoubleConstructor() {
		assertEquals("name", o1.getName());
		assertEquals(1.5, o1.getWidth(), 0);
		assertEquals(3.0, o1.getHeight(), 0);

		assertEquals("{\"name\":\"name\",\"width\":1.5,\"height\":3.0}",
				o1.toString());

		assertEquals(3, o1.toJsonObject().size());
		assertEquals("name", o1.toJsonObject().getString(Item.FIELD_NAME));
		assertEquals(1.5, o1.toJsonObject().getJsonNumber(Item.FIELD_WIDTH)
				.doubleValue(), 0);
		assertEquals(3.0, o1.toJsonObject().getJsonNumber(Item.FIELD_HEIGHT)
				.doubleValue(), 0);

		assertEquals(Item.TYPE, o1.getMessageType());
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
	}

	@Test
	public void testEquals() {
		assertFalse(empty.equals(o1));
		assertFalse(o1.equals(empty));

		assertTrue(o1.equals(o1));
		assertTrue(empty.equals(empty));
	}

	@Test
	public void testEqualsWrongObject() {
		assertFalse(o1.equals(new String(o1.toString())));
	}

	@Test
	public void testClone() {
		Item clone = o1.clone();
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
		Item p = Item.fromJsonString(o1.toString());
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
		Item p = Item.fromMessage(m);
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
				.add(Item.FIELD_NAME, o1.getName())
				.add(Item.FIELD_WIDTH, o1.getWidth())
				.add(Item.FIELD_HEIGHT, o1.getHeight()).build();
		Item p = Item.fromJsonObject(jsonObject);
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
				.add(Item.FIELD_WIDTH, o1.getWidth())
				.add(Item.FIELD_HEIGHT, o1.getHeight()).build();
		Item p = Item.fromJsonObject(jsonObject);
		assertEquals("", p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
	}

	@Test
	public void testFromJsonObjectNoWidth() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Item.FIELD_NAME, o1.getName())
				.add(Item.FIELD_HEIGHT, o1.getHeight()).build();
		Item p = Item.fromJsonObject(jsonObject);
		assertEquals(o1.getName(), p.getName());
		assertEquals(0.0, p.getWidth(), 0);
		assertEquals(o1.getHeight(), p.getHeight(), 0);
	}

	@Test
	public void testFromJsonObjectNoHeight() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Item.FIELD_NAME, o1.getName())
				.add(Item.FIELD_WIDTH, o1.getWidth()).build();
		Item p = Item.fromJsonObject(jsonObject);
		assertEquals(o1.getName(), p.getName());
		assertEquals(o1.getWidth(), p.getWidth(), 0);
		assertEquals(0.0, p.getHeight(), 0);
	}
}
