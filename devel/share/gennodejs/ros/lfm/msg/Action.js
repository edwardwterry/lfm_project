// Auto-generated. Do not edit!

// (in-package lfm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Action {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_tag = null;
      this.dist = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('target_tag')) {
        this.target_tag = initObj.target_tag
      }
      else {
        this.target_tag = 0;
      }
      if (initObj.hasOwnProperty('dist')) {
        this.dist = initObj.dist
      }
      else {
        this.dist = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Action
    // Serialize message field [target_tag]
    bufferOffset = _serializer.int32(obj.target_tag, buffer, bufferOffset);
    // Serialize message field [dist]
    bufferOffset = _serializer.float32(obj.dist, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Action
    let len;
    let data = new Action(null);
    // Deserialize message field [target_tag]
    data.target_tag = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [dist]
    data.dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lfm/Action';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7529da8ba0ed13c6f3938479d680da6a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 target_tag
    float32 dist
    float32 angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Action(null);
    if (msg.target_tag !== undefined) {
      resolved.target_tag = msg.target_tag;
    }
    else {
      resolved.target_tag = 0
    }

    if (msg.dist !== undefined) {
      resolved.dist = msg.dist;
    }
    else {
      resolved.dist = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    return resolved;
    }
};

module.exports = Action;
