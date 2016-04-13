package com.roboclub.robobuggy.simulation;

import java.util.Date;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommandMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * utilities for playback
 */
public class PlayBackUtil {
	  private static final String METADATA_NAME = "Robobuggy Data Logs";
	  private static final String METADATA_SCHEMA_VERSION = "1.1";
	  private static final String METADATA_HIGHLEVEL_SW_VERSION = "1.0.0";
	  private static final  Publisher imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
	  private static final Publisher  magPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());;
	  private static final Publisher  gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());;
	  private static final  Publisher encoderPub  = new Publisher(NodeChannel.ENCODER.getMsgPath());
	  private static final Publisher  brakePub = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
	  private static final Publisher  steeringPub = new Publisher(NodeChannel.STEERING_COMMANDED.getMsgPath());
	  private static final Publisher  loggingButtonPub  = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
	  private static final Publisher  logicNotificationPub = new Publisher(NodeChannel.LOGIC_NOTIFICATION.getMsgPath());



    /**
     * validates the log file metadata
     * @param logFile the log file to validate
     * @return whether or not the log file is valid
     */
    public static boolean validateLogFileMetadata(JsonObject logFile) {
    	if(logFile == null){
    		return false;
    	}
    	
        if (!logFile.get("name").getAsString().equals(METADATA_NAME)) {
            return false;
        }
        if (!logFile.get("schema_version").getAsString().equals(METADATA_SCHEMA_VERSION)) {
            return false;
        }
        if (!logFile.get("software_version").getAsString().equals(METADATA_HIGHLEVEL_SW_VERSION)) {
            return false;
        }

        return true;
    }
    

    
    public static Message parseSensorLog(JsonObject sensorDataJson,Gson translator,long playBacktime,long sensorStartTime,double playBackSpeed) throws InterruptedException{;
            String versionID = sensorDataJson.get("VERSION_ID").getAsString();
			long sensorTime = sensorDataJson.get("timestamp").getAsLong();
            Message transmitMessage = null;
            switch (versionID) {
                case BrakeControlMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, BrakeControlMessage.class);
                    break;
                case BrakeMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, BrakeMessage.class);
                    break;
                case MagneticMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, MagneticMeasurement.class);
                    break;
                case DriveControlMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, DriveControlMessage.class);
                    break;
                case EncoderMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, EncoderMeasurement.class);
                    break;
                case FingerPrintMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, FingerPrintMessage.class);
                    break;
                case GpsMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GpsMeasurement.class);
                    break;
                case GuiLoggingButtonMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GuiLoggingButtonMessage.class);
                    break;
                case ImuMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, ImuMeasurement.class);
                    break;
                case GPSPoseMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, GPSPoseMessage.class);
                    break;
                case RemoteWheelAngleRequest.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, RemoteWheelAngleRequest.class);
                    break;
                case ResetMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, ResetMessage.class);
                    break;
                case RobobuggyLogicNotificationMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, RobobuggyLogicNotificationMeasurement.class);
                    break;
                case StateMessage.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, StateMessage.class);
                    break;
                case SteeringMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, SteeringMeasurement.class);
                    break;
                case WheelAngleCommandMeasurement.VERSION_ID:
                    transmitMessage = translator.fromJson(sensorDataJson, WheelAngleCommandMeasurement.class);
                    break;
             /*   case TERMINATING_VERSION_ID:
                    new RobobuggyLogicNotification("Stopping playback, hit a STOP", RobobuggyMessageLevel.NOTE);
                    return;
                    */
                default:
                    break;
            }
            long sensorDt = (sensorTime-sensorStartTime);
            long dt = (long) (sensorDt/playBackSpeed) - playBacktime;
            if(dt> 10){ //Milliseconds
            	Thread.sleep(dt);
            }
            	
            
            //actually send the message  //TODO stop from using version id, should use topic instead
            switch (versionID) {
            case BrakeMessage.VERSION_ID:
                brakePub.publish(transmitMessage);
                break;
            case EncoderMeasurement.VERSION_ID:
                encoderPub.publish(transmitMessage);
                break;
            case GpsMeasurement.VERSION_ID:
                gpsPub.publish(transmitMessage);
                break;
            case GuiLoggingButtonMessage.VERSION_ID:
                loggingButtonPub.publish(transmitMessage);
                break;
            case ImuMeasurement.VERSION_ID:
                imuPub.publish(transmitMessage);
                break;
            case RobobuggyLogicNotificationMeasurement.VERSION_ID:
                logicNotificationPub.publish(transmitMessage);
                break;
            case SteeringMeasurement.VERSION_ID:
                steeringPub.publish(transmitMessage);
                break;
            case MagneticMeasurement.VERSION_ID:
                magPub.publish(transmitMessage);
                break;
            default:
                break;
        }
			return transmitMessage;
    }
	
}
