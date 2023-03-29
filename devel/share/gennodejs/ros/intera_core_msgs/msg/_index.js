
"use strict";

let IODeviceStatus = require('./IODeviceStatus.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let IODataStatus = require('./IODataStatus.js');
let DigitalIOState = require('./DigitalIOState.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let HeadState = require('./HeadState.js');
let JointLimits = require('./JointLimits.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let JointCommand = require('./JointCommand.js');
let AnalogIOState = require('./AnalogIOState.js');
let NavigatorState = require('./NavigatorState.js');
let IONodeStatus = require('./IONodeStatus.js');
let CameraSettings = require('./CameraSettings.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let CameraControl = require('./CameraControl.js');
let HomingCommand = require('./HomingCommand.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let EndpointStates = require('./EndpointStates.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let NavigatorStates = require('./NavigatorStates.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let HomingState = require('./HomingState.js');
let IOStatus = require('./IOStatus.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let SEAJointState = require('./SEAJointState.js');
let IONodeConfiguration = require('./IONodeConfiguration.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let EndpointState = require('./EndpointState.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let InteractionControlState = require('./InteractionControlState.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');

module.exports = {
  IODeviceStatus: IODeviceStatus,
  HeadPanCommand: HeadPanCommand,
  IODataStatus: IODataStatus,
  DigitalIOState: DigitalIOState,
  IODeviceConfiguration: IODeviceConfiguration,
  IOComponentConfiguration: IOComponentConfiguration,
  HeadState: HeadState,
  JointLimits: JointLimits,
  AnalogOutputCommand: AnalogOutputCommand,
  DigitalOutputCommand: DigitalOutputCommand,
  JointCommand: JointCommand,
  AnalogIOState: AnalogIOState,
  NavigatorState: NavigatorState,
  IONodeStatus: IONodeStatus,
  CameraSettings: CameraSettings,
  CollisionAvoidanceState: CollisionAvoidanceState,
  DigitalIOStates: DigitalIOStates,
  CollisionDetectionState: CollisionDetectionState,
  CameraControl: CameraControl,
  HomingCommand: HomingCommand,
  AnalogIOStates: AnalogIOStates,
  EndpointStates: EndpointStates,
  URDFConfiguration: URDFConfiguration,
  NavigatorStates: NavigatorStates,
  IOComponentCommand: IOComponentCommand,
  HomingState: HomingState,
  IOStatus: IOStatus,
  IOComponentStatus: IOComponentStatus,
  SEAJointState: SEAJointState,
  IONodeConfiguration: IONodeConfiguration,
  InteractionControlCommand: InteractionControlCommand,
  RobotAssemblyState: RobotAssemblyState,
  EndpointState: EndpointState,
  EndpointNamesArray: EndpointNamesArray,
  InteractionControlState: InteractionControlState,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandResult: CalibrationCommandResult,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
  CalibrationCommandAction: CalibrationCommandAction,
};
