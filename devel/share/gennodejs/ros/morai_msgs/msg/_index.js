
"use strict";

let VehicleCollisionData = require('./VehicleCollisionData.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let ERP42Info = require('./ERP42Info.js');
let WaitForTick = require('./WaitForTick.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let TrafficLight = require('./TrafficLight.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let GhostMessage = require('./GhostMessage.js');
let MapSpec = require('./MapSpec.js');
let ObjectStatus = require('./ObjectStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let CollisionData = require('./CollisionData.js');
let SensorPosControl = require('./SensorPosControl.js');
let GPSMessage = require('./GPSMessage.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let IntscnTL = require('./IntscnTL.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let EventInfo = require('./EventInfo.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let VehicleSpec = require('./VehicleSpec.js');
let RadarDetection = require('./RadarDetection.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let PRStatus = require('./PRStatus.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let IntersectionControl = require('./IntersectionControl.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let SaveSensorData = require('./SaveSensorData.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let SVADC = require('./SVADC.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let DillyCmd = require('./DillyCmd.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let RadarDetections = require('./RadarDetections.js');
let ReplayInfo = require('./ReplayInfo.js');
let Lamps = require('./Lamps.js');
let PREvent = require('./PREvent.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let CtrlCmd = require('./CtrlCmd.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SyncModeCmd = require('./SyncModeCmd.js');

module.exports = {
  VehicleCollisionData: VehicleCollisionData,
  DdCtrlCmd: DdCtrlCmd,
  ERP42Info: ERP42Info,
  WaitForTick: WaitForTick,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  SyncModeResultResponse: SyncModeResultResponse,
  TrafficLight: TrafficLight,
  VehicleSpecIndex: VehicleSpecIndex,
  MultiPlayEventRequest: MultiPlayEventRequest,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  GhostMessage: GhostMessage,
  MapSpec: MapSpec,
  ObjectStatus: ObjectStatus,
  VehicleCollision: VehicleCollision,
  CollisionData: CollisionData,
  SensorPosControl: SensorPosControl,
  GPSMessage: GPSMessage,
  IntersectionStatus: IntersectionStatus,
  IntscnTL: IntscnTL,
  SyncModeInfo: SyncModeInfo,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  SyncModeRemoveObject: SyncModeRemoveObject,
  SyncModeAddObject: SyncModeAddObject,
  MoraiSimProcStatus: MoraiSimProcStatus,
  EventInfo: EventInfo,
  SyncModeSetGear: SyncModeSetGear,
  ObjectStatusListExtended: ObjectStatusListExtended,
  MoraiTLInfo: MoraiTLInfo,
  EgoVehicleStatus: EgoVehicleStatus,
  NpcGhostInfo: NpcGhostInfo,
  NpcGhostCmd: NpcGhostCmd,
  SyncModeCmdResponse: SyncModeCmdResponse,
  WoowaDillyStatus: WoowaDillyStatus,
  MapSpecIndex: MapSpecIndex,
  VehicleSpec: VehicleSpec,
  RadarDetection: RadarDetection,
  MoraiSimProcHandle: MoraiSimProcHandle,
  PRStatus: PRStatus,
  MoraiTLIndex: MoraiTLIndex,
  IntersectionControl: IntersectionControl,
  WaitForTickResponse: WaitForTickResponse,
  ObjectStatusList: ObjectStatusList,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  SaveSensorData: SaveSensorData,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  SVADC: SVADC,
  MultiPlayEventResponse: MultiPlayEventResponse,
  SkateboardStatus: SkateboardStatus,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  SetTrafficLight: SetTrafficLight,
  DillyCmd: DillyCmd,
  ScenarioLoad: ScenarioLoad,
  PRCtrlCmd: PRCtrlCmd,
  MoraiSrvResponse: MoraiSrvResponse,
  RadarDetections: RadarDetections,
  ReplayInfo: ReplayInfo,
  Lamps: Lamps,
  PREvent: PREvent,
  MultiEgoSetting: MultiEgoSetting,
  DillyCmdResponse: DillyCmdResponse,
  CtrlCmd: CtrlCmd,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  ObjectStatusExtended: ObjectStatusExtended,
  SyncModeCmd: SyncModeCmd,
};
