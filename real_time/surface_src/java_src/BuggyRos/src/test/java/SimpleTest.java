import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class SimpleTest {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void onePubOneSub() {
		int numIterations = 1000000;
		NumberSink sink = new NumberSink("to", numIterations);
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("to", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
		System.out.println("MainThread exiting...");
	}

	@Test
	public void onePubOneSubWithMiddleNode() {
		int numIterations = 1000000;
		NumberSink sink = new NumberSink("to", numIterations);
		MessagePasser ms = new MessagePasser("from", "to");
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("from", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
		System.out.println("MainThread exiting...");
	}
}