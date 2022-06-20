// Auto-generated. Do not edit!

// (in-package mpc_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TaskControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.traj_end = null;
      this.traj_turn = null;
    }
    else {
      if (initObj.hasOwnProperty('traj_end')) {
        this.traj_end = initObj.traj_end
      }
      else {
        this.traj_end = 0;
      }
      if (initObj.hasOwnProperty('traj_turn')) {
        this.traj_turn = initObj.traj_turn
      }
      else {
        this.traj_turn = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskControl
    // Serialize message field [traj_end]
    bufferOffset = _serializer.uint8(obj.traj_end, buffer, bufferOffset);
    // Serialize message field [traj_turn]
    bufferOffset = _serializer.uint8(obj.traj_turn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskControl
    let len;
    let data = new TaskControl(null);
    // Deserialize message field [traj_end]
    data.traj_end = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [traj_turn]
    data.traj_turn = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_msgs/TaskControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc711d3525457fc9a0066f50f991e5eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 traj_end
    uint8 traj_turn
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TaskControl(null);
    if (msg.traj_end !== undefined) {
      resolved.traj_end = msg.traj_end;
    }
    else {
      resolved.traj_end = 0
    }

    if (msg.traj_turn !== undefined) {
      resolved.traj_turn = msg.traj_turn;
    }
    else {
      resolved.traj_turn = 0
    }

    return resolved;
    }
};

module.exports = TaskControl;
