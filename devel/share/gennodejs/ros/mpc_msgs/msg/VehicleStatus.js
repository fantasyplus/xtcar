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

class VehicleStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_mode = null;
      this.error_level = null;
      this.door_state = null;
      this.lamp_L = null;
      this.lamp_R = null;
      this.voltage = null;
      this.current = null;
      this.steer = null;
      this.hand_brake = null;
      this.gear = null;
      this.stop = null;
      this.acc = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('car_mode')) {
        this.car_mode = initObj.car_mode
      }
      else {
        this.car_mode = 0;
      }
      if (initObj.hasOwnProperty('error_level')) {
        this.error_level = initObj.error_level
      }
      else {
        this.error_level = 0;
      }
      if (initObj.hasOwnProperty('door_state')) {
        this.door_state = initObj.door_state
      }
      else {
        this.door_state = 0;
      }
      if (initObj.hasOwnProperty('lamp_L')) {
        this.lamp_L = initObj.lamp_L
      }
      else {
        this.lamp_L = 0;
      }
      if (initObj.hasOwnProperty('lamp_R')) {
        this.lamp_R = initObj.lamp_R
      }
      else {
        this.lamp_R = 0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('steer')) {
        this.steer = initObj.steer
      }
      else {
        this.steer = 0.0;
      }
      if (initObj.hasOwnProperty('hand_brake')) {
        this.hand_brake = initObj.hand_brake
      }
      else {
        this.hand_brake = 0;
      }
      if (initObj.hasOwnProperty('gear')) {
        this.gear = initObj.gear
      }
      else {
        this.gear = 0;
      }
      if (initObj.hasOwnProperty('stop')) {
        this.stop = initObj.stop
      }
      else {
        this.stop = 0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleStatus
    // Serialize message field [car_mode]
    bufferOffset = _serializer.uint8(obj.car_mode, buffer, bufferOffset);
    // Serialize message field [error_level]
    bufferOffset = _serializer.uint8(obj.error_level, buffer, bufferOffset);
    // Serialize message field [door_state]
    bufferOffset = _serializer.uint8(obj.door_state, buffer, bufferOffset);
    // Serialize message field [lamp_L]
    bufferOffset = _serializer.uint8(obj.lamp_L, buffer, bufferOffset);
    // Serialize message field [lamp_R]
    bufferOffset = _serializer.uint8(obj.lamp_R, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float32(obj.current, buffer, bufferOffset);
    // Serialize message field [steer]
    bufferOffset = _serializer.float32(obj.steer, buffer, bufferOffset);
    // Serialize message field [hand_brake]
    bufferOffset = _serializer.uint8(obj.hand_brake, buffer, bufferOffset);
    // Serialize message field [gear]
    bufferOffset = _serializer.uint8(obj.gear, buffer, bufferOffset);
    // Serialize message field [stop]
    bufferOffset = _serializer.uint8(obj.stop, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = _serializer.float32(obj.acc, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleStatus
    let len;
    let data = new VehicleStatus(null);
    // Deserialize message field [car_mode]
    data.car_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error_level]
    data.error_level = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [door_state]
    data.door_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lamp_L]
    data.lamp_L = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lamp_R]
    data.lamp_R = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steer]
    data.steer = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hand_brake]
    data.hand_brake = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gear]
    data.gear = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stop]
    data.stop = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_msgs/VehicleStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b163f7630993eb80c9c2c58fe427804d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 car_mode
    uint8 error_level
    uint8 door_state
    uint8 lamp_L
    uint8 lamp_R
    float32 voltage
    float32 current
    float32 steer
    uint8 hand_brake
    uint8 gear
    uint8 stop
    float32 acc
    float32 speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehicleStatus(null);
    if (msg.car_mode !== undefined) {
      resolved.car_mode = msg.car_mode;
    }
    else {
      resolved.car_mode = 0
    }

    if (msg.error_level !== undefined) {
      resolved.error_level = msg.error_level;
    }
    else {
      resolved.error_level = 0
    }

    if (msg.door_state !== undefined) {
      resolved.door_state = msg.door_state;
    }
    else {
      resolved.door_state = 0
    }

    if (msg.lamp_L !== undefined) {
      resolved.lamp_L = msg.lamp_L;
    }
    else {
      resolved.lamp_L = 0
    }

    if (msg.lamp_R !== undefined) {
      resolved.lamp_R = msg.lamp_R;
    }
    else {
      resolved.lamp_R = 0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.steer !== undefined) {
      resolved.steer = msg.steer;
    }
    else {
      resolved.steer = 0.0
    }

    if (msg.hand_brake !== undefined) {
      resolved.hand_brake = msg.hand_brake;
    }
    else {
      resolved.hand_brake = 0
    }

    if (msg.gear !== undefined) {
      resolved.gear = msg.gear;
    }
    else {
      resolved.gear = 0
    }

    if (msg.stop !== undefined) {
      resolved.stop = msg.stop;
    }
    else {
      resolved.stop = 0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleStatus;
