package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used for keeping track of when the encoder has been reset
 *
 * @author Sean
 *         <p>
 *         CHANGELOG: NONE
 *         <p>
 *         DESCRIPTION: TODO
 * @version 0.5
 */
public class EncoderResetMessage extends BaseMessage {

    public static final String VERSION_ID = "EncoderResetV0.1";

    /**
     * Construct a new {@link EncoderResetMessage} at time now
     */
    public EncoderResetMessage() {
        this.timestamp = new Date().getTime();
    }

    /**
     * Construct a new {@link EncoderResetMessage}
     *
     * @param timestamp {@link Date} representing the time of the message
     */
    public EncoderResetMessage(Date timestamp) {
        this.timestamp = new Date(timestamp.getTime()).getTime();
    }

}