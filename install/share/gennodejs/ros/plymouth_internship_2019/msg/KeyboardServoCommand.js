// Auto-generated. Do not edit!

// (in-package plymouth_internship_2019.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class KeyboardServoCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_command_1 = null;
      this.servo_command_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_command_1')) {
        this.servo_command_1 = initObj.servo_command_1
      }
      else {
        this.servo_command_1 = 0;
      }
      if (initObj.hasOwnProperty('servo_command_2')) {
        this.servo_command_2 = initObj.servo_command_2
      }
      else {
        this.servo_command_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KeyboardServoCommand
    // Serialize message field [servo_command_1]
    bufferOffset = _serializer.int64(obj.servo_command_1, buffer, bufferOffset);
    // Serialize message field [servo_command_2]
    bufferOffset = _serializer.int64(obj.servo_command_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KeyboardServoCommand
    let len;
    let data = new KeyboardServoCommand(null);
    // Deserialize message field [servo_command_1]
    data.servo_command_1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [servo_command_2]
    data.servo_command_2 = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plymouth_internship_2019/KeyboardServoCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9dbce863de6b75635fc5625ec0af5414';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 servo_command_1
    int64 servo_command_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KeyboardServoCommand(null);
    if (msg.servo_command_1 !== undefined) {
      resolved.servo_command_1 = msg.servo_command_1;
    }
    else {
      resolved.servo_command_1 = 0
    }

    if (msg.servo_command_2 !== undefined) {
      resolved.servo_command_2 = msg.servo_command_2;
    }
    else {
      resolved.servo_command_2 = 0
    }

    return resolved;
    }
};

module.exports = KeyboardServoCommand;
