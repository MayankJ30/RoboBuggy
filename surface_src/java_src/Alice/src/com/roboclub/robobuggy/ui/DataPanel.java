package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class DataPanel extends JPanel {
	private static final long serialVersionUID = 3950373392222628865L;

	private GpsPanel gpsPanel;
	
	/* Data Fields */
	private JLabel aX, aY, aZ;
	private JLabel rX, rY, rZ;
	private JLabel mX, mY, mZ;
	private JLabel encTicks;
	private JLabel steeringAng;
	private JLabel errorNum;
	
	public DataPanel() {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridBagLayout());
		
		GridBagConstraints gbc = new GridBagConstraints();
		
		gbc.fill = GridBagConstraints.BOTH;
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.weightx = 0.34;
		gbc.weighty = 1.0;
		gpsPanel = new GpsPanel();
		this.add(gpsPanel, gbc);
		
		gbc.gridx = 1;
		gbc.weightx = 0.66;
		this.add(createDataPanel(), gbc);
	}
	
	private JPanel createDataPanel() {
		JPanel panel = new JPanel();
		panel.setBorder(BorderFactory.createLineBorder(Color.black));
		panel.setLayout(new GridLayout(4,6));
		
		aX = new JLabel();
		JLabel label = new JLabel("   aX: ");
		panel.add(label);
		panel.add(aX);
		
		aY = new JLabel();
		label = new JLabel("   aY: ");
		panel.add(label);
		panel.add(aY);
		
		aZ = new JLabel();
		label = new JLabel("   aZ: ");
		panel.add(label);
		panel.add(aZ);
		
		rX = new JLabel();
		label = new JLabel("   rX: ");
		panel.add(label);
		panel.add(rX);
		
		rY = new JLabel();
		label = new JLabel("   rY: ");
		panel.add(label);
		panel.add(rY);
		
		rZ = new JLabel();
		label = new JLabel("   rZ: ");
		panel.add(label);
		panel.add(rZ);
		
		mX = new JLabel();
		label = new JLabel("   mX: ");
		panel.add(label);
		panel.add(mX);
		
		mY = new JLabel();
		label = new JLabel("   mY: ");
		panel.add(label);
		panel.add(mY);
		
		mZ = new JLabel();
		label = new JLabel("   mZ: ");
		panel.add(label);
		panel.add(mZ);
		
		// Subscriber for Imu updates
		new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				ImuMeasurement tmp = (ImuMeasurement)m;
				
				// Limit measurement values to 10 characters
				aX.setText(Double.toString(tmp.aX).substring(0, 10));
				aY.setText(Double.toString(tmp.aY).substring(0, 10));
				aZ.setText(Double.toString(tmp.aZ).substring(0, 10));
				rX.setText(Double.toString(tmp.rX).substring(0, 10));
				rY.setText(Double.toString(tmp.rY).substring(0, 10));
				rZ.setText(Double.toString(tmp.rZ).substring(0, 10));
				mX.setText(Double.toString(tmp.mX).substring(0, 10));
				mY.setText(Double.toString(tmp.mY).substring(0, 10));
				mZ.setText(Double.toString(tmp.mZ).substring(0, 10));
			}
		});
		
		encTicks = new JLabel();
		label = new JLabel("   Ticks: ");
		panel.add(label);
		panel.add(encTicks);
		
		// Subscriber for encoder updates
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				encTicks.setText(Double.toString(((EncoderMeasurement)m).distance));
			}
		});
		
		steeringAng = new JLabel();
		label = new JLabel("   Angle: ");
		panel.add(label);
		panel.add(steeringAng);
		
		// Subscriber for drive control updates
		new Subscriber(SensorChannel.DRIVE_CTRL.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringAng.setText(Integer.toString(((WheelAngleCommand)m).angle));
			}
		});
		
		errorNum = new JLabel();
		label = new JLabel("   Errors: ");
		panel.add(label);
		panel.add(errorNum);
		
		return panel;
	}
}
