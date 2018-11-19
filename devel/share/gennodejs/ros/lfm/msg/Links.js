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

class Links {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_id = null;
      this.link_active_prob = null;
    }
    else {
      if (initObj.hasOwnProperty('link_id')) {
        this.link_id = initObj.link_id
      }
      else {
        this.link_id = [];
      }
      if (initObj.hasOwnProperty('link_active_prob')) {
        this.link_active_prob = initObj.link_active_prob
      }
      else {
        this.link_active_prob = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Links
    // Serialize message field [link_id]
    bufferOffset = _arraySerializer.int32(obj.link_id, buffer, bufferOffset, null);
    // Serialize message field [link_active_prob]
    bufferOffset = _arraySerializer.float32(obj.link_active_prob, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Links
    let len;
    let data = new Links(null);
    // Deserialize message field [link_id]
    data.link_id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [link_active_prob]
    data.link_active_prob = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.link_id.length;
    length += 4 * object.link_active_prob.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lfm/Links';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20b3840007e10c6883a36f0694bd3f6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] link_id
    float32[] link_active_prob
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Links(null);
    if (msg.link_id !== undefined) {
      resolved.link_id = msg.link_id;
    }
    else {
      resolved.link_id = []
    }

    if (msg.link_active_prob !== undefined) {
      resolved.link_active_prob = msg.link_active_prob;
    }
    else {
      resolved.link_active_prob = []
    }

    return resolved;
    }
};

module.exports = Links;
