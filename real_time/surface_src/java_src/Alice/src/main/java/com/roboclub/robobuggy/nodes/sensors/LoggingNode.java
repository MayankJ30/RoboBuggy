package com.roboclub.robobuggy.nodes.sensors;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.BaseMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.PoseMessage;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;

/**
 * {@link SerialNode} for reading in logging commands from the GUI
 * When logging begins, a new folder is created, and then logging begins
 *  to that folder
 */
public class LoggingNode extends BuggyDecoratorNode {

	private Publisher loggingButtonPub;
	private File outputFile;
    private File outputDirectory;
    private NodeChannel[] filters;
    private LinkedList<Message> messageQueue;
    private LogWriterRunnable loggingThread;
    private boolean keepLogging;

	/**
	 * Create a new {@link LoggingNode} decorator
	 * @param channel the {@link NodeChannel} of the {@link LoggingNode}
     * @param outputDirPath The path to the output directory (not file)
     * @param filters sensors to log. To log all sensors, just use NodeChannel.values()
	 */
	public LoggingNode(NodeChannel channel, String outputDirPath, NodeChannel...filters) {
		super(new BuggyBaseNode(channel));

        this.filters = filters;
        messageQueue = new LinkedList<>();
        keepLogging = true;
        loggingThread = new LogWriterRunnable();

        createNewLogFile(outputDirPath);
        setupSubscriberList();
        setupLoggingTrigger();

    }



    /**
     * Starts the logging process
     */
    private void setupLoggingTrigger() {
        new Subscriber(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GuiLoggingButtonMessage message = (GuiLoggingButtonMessage) m;
                if (message.getLoggingMessage().equals(GuiLoggingButtonMessage.LoggingMessage.START)) {
                    createNewLogFile(outputDirectory.getPath());
                    keepLogging = true;
                    loggingThread.start();
                    new RobobuggyLogicNotification("Starting up logging thread!", RobobuggyMessageLevel.NOTE);
                }
                else if (message.getLoggingMessage().equals(GuiLoggingButtonMessage.LoggingMessage.STOP)) {
                    keepLogging = false;
                    new RobobuggyLogicNotification("Stopping logging thread!", RobobuggyMessageLevel.NOTE);
                }
                else {
                    new RobobuggyLogicNotification("Gui said something logger couldn't understand!", RobobuggyMessageLevel.EXCEPTION);
                }
            }
        });
    }

    /**
     * Sets up the subscriber list - Simply enumerates over our NodeChannel filters and adds
     * a subscriber for each one
     */
    private void setupSubscriberList() {
        for (NodeChannel filter : filters) {
            new Subscriber(filter.getMsgPath(), new MessageListener() {
                @Override
                public void actionPerformed(String topicName, Message m) {
                    messageQueue.add(m);
                }
            });
        }
    }


    /**
     * Creates the log file, and returns the status
     * Returns false if anything went wrong, but already throws the logic exception
     *
     * @param outputDirString the directory string of the log file
     * @return the status of the operation - true if it succeeded, false if it didn't
     */
    private boolean createNewLogFile(String outputDirString) {
        outputDirectory = new File(outputDirString);
        if (!outputDirectory.exists() || !outputDirectory.isDirectory()) {
            new RobobuggyLogicNotification("Output directory path isn't a folder!", RobobuggyMessageLevel.EXCEPTION);
            return false;
        }

        // each log file is called {filename}_{date}.txt
        outputFile = new File(outputDirectory.getPath() + "/" + RobobuggyConfigFile.LOG_FILE_NAME + "_" + BaseMessage.formatDate(new Date()) + ".txt");
        try {
            if(!outputFile.createNewFile()) {
                new RobobuggyLogicNotification("Couldn't create log file!", RobobuggyMessageLevel.EXCEPTION);
                return false;
            }
        } catch (IOException e) {
            new RobobuggyLogicNotification("Error reading the filesystem!", RobobuggyMessageLevel.EXCEPTION);
            return false;
        }

        //everything succeeded!
        return true;
    }


	/**{@inheritDoc}*/
	@Override
	protected boolean startDecoratorNode() {
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
		
		new Subscriber(Gui.GuiPubSubTopics.GUI_LOG_BUTTON_UPDATED.toString(), new MessageListener() {
			@Override 
			public void actionPerformed(String topicName, Message m) {
				loggingButtonPub.publish(m);
			}
		});
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		return true;
	}


    /**
     * LogWriterRunnable - where we actually process each message and write it to the file
     */
    private class LogWriterRunnable extends Thread {

        private PrintStream fileWriteStream;
        private Gson messageTranslator;

        private int imuHits = 0;
        private int encoderHits = 0;
        private int gpsHits = 0;
        private int brakeHits = 0;
        private int fingerprintHits = 0;
        private int steeringHits = 0;
        private int logicNotificationHits = 0;
        private int logButtonHits = 0;
        private int poseMessageHits = 0;
        private int resetHits = 0;
        private int stateHits = 0;

        private String name = "\"name\": \"Robobuggy Data Logs\",";
        private String schemaVersion = "\"schema_version\": 1.0,";
        private String dateRecorded = "\"date_recorded\": \"" +
                new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date()) + "\",";
        private String swVersion = "\"software_version\": \"" + RobobuggyConfigFile.ALICE_LIBRARY_VERSION + "\",";
        private String sensorDataHeader = "\"sensor_data\": [";
        private String footerDataEntry = "        {\"VERSION_ID\":\"STOP\"}";

        @Override
        public synchronized void start() {

            try {
                fileWriteStream = new PrintStream(outputFile, "UTF-8");
                messageTranslator = new GsonBuilder().excludeFieldsWithModifiers().create();
            } catch (FileNotFoundException | UnsupportedEncodingException e) {
                new RobobuggyLogicNotification("Error setting up the output file. Aborting logging!", RobobuggyMessageLevel.EXCEPTION);
                return;
            }

            fileWriteStream.println("{" + "\n    " + name + "\n    " + schemaVersion + "\n    " + dateRecorded
                    + "\n    " + swVersion + "\n    " + sensorDataHeader);

            //always want to log :)
            while (keepLogging) {
                //spin in a loop until a message comes in
                while (messageQueue.isEmpty()) {
                    //TODO be able to spin in a tight loop until the message queue isn't empty
                    // // STOPSHIP: 1/31/16
                    // // FIXME: 1/31/16 spin in a loop
                    stateHits = 0;
                }
                //now we have a message from the queue

                Message toSort = messageQueue.pop();
                String msgAsJsonString = messageTranslator.toJson(toSort);

                // and if you look on your right you'll see the almost-unnecessary
                // giganti-frickin-ic telemetry block
                if (toSort instanceof BrakeMessage) {
                    brakeHits++;
                }
                else if (toSort instanceof EncoderMeasurement) {
                    encoderHits++;
                }
                else if (toSort instanceof FingerPrintMessage) {
                    fingerprintHits++;
                }
                else if (toSort instanceof GpsMeasurement) {
                    gpsHits++;
                }
                else if (toSort instanceof GuiLoggingButtonMessage) {
                    logButtonHits++;
                }
                else if (toSort instanceof ImuMeasurement) {
                    imuHits++;
                }
                else if (toSort instanceof PoseMessage) {
                    poseMessageHits++;
                }
                else if (toSort instanceof ResetMessage) {
                    resetHits++;
                }
                else if (toSort instanceof RobobuggyLogicNotificationMeasurement) {
                    logicNotificationHits++;
                }
                else if (toSort instanceof StateMessage) {
                    stateHits++;
                }
                else if (toSort instanceof SteeringMeasurement) {
                    steeringHits++;
                }
                else {
                    //a new kind of message!
                    new RobobuggyLogicNotification("New message came in that we aren't tracking", RobobuggyMessageLevel.WARNING);
                }

                fileWriteStream.println("        " + msgAsJsonString + ",");

            }

            //we've stopped logging
            JsonObject dataBreakdown = new JsonObject();
            dataBreakdown.addProperty(NodeChannel.GUI_LOGGING_BUTTON.getName(), logButtonHits);
            dataBreakdown.addProperty(NodeChannel.GPS.getName(), gpsHits);
            dataBreakdown.addProperty(NodeChannel.IMU.getName(), imuHits);
            dataBreakdown.addProperty(NodeChannel.ENCODER.getName(), encoderHits);
            dataBreakdown.addProperty(NodeChannel.BRAKE.getName(), brakeHits);
            dataBreakdown.addProperty(NodeChannel.STEERING.getName(), steeringHits);
            dataBreakdown.addProperty(NodeChannel.FP_HASH.getName(), fingerprintHits);
            dataBreakdown.addProperty(NodeChannel.LOGIC_NOTIFICATION.getName(), logicNotificationHits);
            dataBreakdown.addProperty(NodeChannel.POSE.getName(), poseMessageHits);
            dataBreakdown.addProperty(NodeChannel.RESET.getName(), resetHits);
            dataBreakdown.addProperty(NodeChannel.STATE.getName(), stateHits);

            fileWriteStream.println(footerDataEntry);
            fileWriteStream.println("    ],\n    \"data_breakdown\" : " + dataBreakdown.toString() + "\n}");
            fileWriteStream.close();
        }
    }

}
