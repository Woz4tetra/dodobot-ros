// import { resolveSoa } from 'dns';
import React from 'react';
import Switch from "react-switch";
import { TextBoxComponent } from '@syncfusion/ej2-react-inputs';
import { CompressedImage, DodobotBatteryState, DodobotBumperState, DodobotDriveState, DodobotFSRsState, DodobotGripperState, DodobotLinearState } from './Sensors';

var roslib = require('roslib');

// constants
const WS_URL = 'ws://192.168.0.21:9090';
const DEFAULT_LINEAR_VEL = 0.1;
const DEFAULT_ANGULAR_VEL = 1.5;

interface SensorsProps {
}

interface SensorsState {
  ros: any,

  // ros state
  battery?: DodobotBatteryState,
  bumper?: DodobotBumperState,
  drive?: DodobotDriveState,
  fsrs?: DodobotFSRsState,
  gripper?: DodobotGripperState,
  linear?: DodobotLinearState,
  image?: CompressedImage,
  depth?: CompressedImage,
  // other params: DateConstructor
  colorTime: any,
  depthTime: any,
  motors_enabled: boolean,

  robotStateService?: any,
  cmdVelPub?: any,
  tilterCmdPub?: any,
  linear_cmd: any,
  angular_cmd: any,
}

export default class Sensors extends React.Component<SensorsProps,SensorsState> {
  constructor(props:SensorsProps) {
    super(props);

    // setup state variables
    this.state = {
      ros: new roslib.Ros(),
      colorTime: new Date().getTime(),
      depthTime: new Date().getTime(),
      motors_enabled: false,
      linear_cmd: DEFAULT_LINEAR_VEL,
      angular_cmd: DEFAULT_ANGULAR_VEL,
    };

    // bind handlers
    this.batteryCallback = this.batteryCallback.bind(this);
    this.bumperCallback = this.bumperCallback.bind(this);
    this.driveCallback = this.driveCallback.bind(this);
    this.fsrsCallback = this.fsrsCallback.bind(this);
    this.gripperCallback = this.gripperCallback.bind(this);
    this.linearCallback = this.linearCallback.bind(this);
    this.imageCallback = this.imageCallback.bind(this);
    this.depthCallback = this.depthCallback.bind(this);
    this.handleChangeMotorState = this.handleChangeMotorState.bind(this);
    this.MotorsEnabledSection = this.MotorsEnabledSection.bind(this);
    this.handleLinearVelChange = this.handleLinearVelChange.bind(this);
    this.handleAngularVelChange = this.handleAngularVelChange.bind(this);
  }

  handleLinearVelChange(event: any) {
    var value = parseFloat(event.target.value);
    console.log("Setting linear velocity to: " + value);
    this.setState({linear_cmd: value});
  }
  handleAngularVelChange(event: any) {
    var value = parseFloat(event.target.value);
    console.log("Setting angular velocity to: " + value);
    this.setState({angular_cmd: value});
  }

  handleChangeMotorState(motors_enabled: boolean) {
    this.setState({ motors_enabled });
    console.log("Setting motors enabled to " + motors_enabled);

    var request = new roslib.ServiceRequest({active: motors_enabled, reporting: true});
    this.state.robotStateService.callService(request, function (response: any) {        
        console.log('Result for service call set state: ' + response);
    }, function(error: any){
        console.error("Got an error while trying to call set state service: " + error);
    });
  }

  componentDidMount() {
    this.initializeROS();
    this.initializePubsSubs(); 

    // document.addEventListener("click", this._handleDocumentClick, false);
    document.addEventListener("keydown", this.handleKeyDown);
    document.addEventListener("keyup", this.handleKeyUp);
  }

  componentWillUnmount() {
    this.state.ros.close();

    // document.removeEventListener("click", this._handleDocumentClick, false);
    document.removeEventListener("keydown", this.handleKeyDown);
    document.removeEventListener("keyup", this.handleKeyUp);
  }

  initializeROS() {
    this.state.ros.on('error', (error: any) => console.log(error));
    this.state.ros.on('connection', (error: any) => console.log('Connection made!'));
    this.state.ros.on('close', (error: any) => console.log('Connection closed: ' + error));
    this.state.ros.connect(WS_URL);

    console.log('Hello!');
  }

  initializePubsSubs() {
    // chatter sub
    var batterySub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/battery',
      messageType: 'sensor_msgs/BatteryState'
    });

    var bumperSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/bumper',
      messageType: 'db_parsing/DodobotBumper'
    });

    var driveSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/drive',
      messageType: 'db_parsing/DodobotDrive'
    });

    var fsrsSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/fsrs',
      messageType: 'db_parsing/DodobotFSRs'
    });

    var gripperSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/gripper',
      messageType: 'db_parsing/DodobotGripper'
    });

    var linearSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/linear',
      messageType: 'db_parsing/DodobotLinear'
    });

    var imageSub = new roslib.Topic({
      ros: this.state.ros,
      // name: '/camera/color/image_raw/compressed',
      name: '/camera/color/image_thumb_raw/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });
    var depthSub = new roslib.Topic({
      ros: this.state.ros,
      name: '/camera/depth/image_color_raw/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

    var cmdVelPub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
    this.setState({cmdVelPub});

    var tilterCmdPub = new roslib.Topic({
      ros: this.state.ros,
      name: '/dodobot/tilter_cmd',
      messageType: 'db_parsing/DodobotTilter'
    });
    this.setState({tilterCmdPub});

    var robotStateService = new roslib.Service({
      ros: this.state.ros,
      name: '/dodobot/set_state',
      serviceType: 'db_parsing/DodobotSetState'
    });

    this.setState({robotStateService});


    batterySub.subscribe(this.batteryCallback);
    bumperSub.subscribe(this.bumperCallback);
    driveSub.subscribe(this.driveCallback);
    fsrsSub.subscribe(this.fsrsCallback);
    gripperSub.subscribe(this.gripperCallback);
    linearSub.subscribe(this.linearCallback);
    imageSub.subscribe(this.imageCallback);
    depthSub.subscribe(this.depthCallback);
  }

  // ROS Callback Functions
  batteryCallback(message: any) {
    this.setState({
      battery: {
        voltage: message.voltage,
        current: message.current
      }
    });
  }

  bumperCallback(message: any) {
    this.setState({
      bumper: {
        left: message.left,
        right: message.right
      }
    });
  }

  driveCallback(message: any) {
    this.setState({
      drive: {
        left_setpoint: message.left_setpoint,
        right_setpoint: message.right_setpoint,
        left_enc_pos: message.left_enc_pos,
        right_enc_pos: message.right_enc_pos,
        left_enc_speed: message.left_enc_speed,
        right_enc_speed: message.right_enc_speed,
        left_bumper: message.left_bumper,
        right_bumper: message.right_bumper
      }
    });
  }

  fsrsCallback(message: any) {
    this.setState({
      fsrs: {
        left: message.left,
        right: message.right
      }
    });
  }

  gripperCallback(message: any) {
    this.setState({
      gripper: {
        position: message.position,
        force_threshold: message.force_threshold
      }
    });
  }

  linearCallback(message: any) {
    this.setState({
      linear: {
        position: message.position,
        has_error: message.has_error,
        is_homed: message.is_homed,
        is_active: message.is_active,
        command_type: message.command_type,
        command_value: message.command_value,
        max_speed: message.max_speed,
        acceleration: message.acceleration
      }
    })
  }
  depthCallback(message: any) {
    const imgSrc = "data:image/jpg;base64, " + message.data;

    const newTime = new Date().getTime();
    const fps = 1000 / (newTime - this.state.depthTime);
    
    this.setState({
      depth: {
        format: message.format,
        // data: message.data,
        imageSrc: imgSrc,
        fps: fps
      },

      depthTime: newTime
    });
  }

  imageCallback(message: any) {
    const imgSrc = "data:image/jpg;base64, " + message.data;

    const newTime = new Date().getTime();
    const fps = 1000 / (newTime - this.state.colorTime);
    
    this.setState({
      image: {
        format: message.format,
        // data: message.data,
        imageSrc: imgSrc,
        fps: fps
      },

      colorTime: newTime
    });
  }


  handleKeyDown = (event: any) => {
    if (this.state.motors_enabled) {
      var twist = new roslib.Message({
        linear : {
          x : 0.0,
          y : 0.0,
          z : 0.0
        },
        angular : {
          x : 0.0,
          y : 0.0,
          z : 0.0
        }
      });

      switch (event.key) {
        case "w":
          twist.linear.x = this.state.linear_cmd;
          console.log("Driving at " + twist.linear.x + " m/s");
          break;
        case "a":
          twist.angular.z = this.state.angular_cmd;
          console.log("Rotating at " + twist.angular.z + " rad/s");
          break;
        case "s":
          twist.linear.x = -this.state.linear_cmd;
          console.log("Driving at " + twist.linear.x + " m/s");
          break;
        case "d":
          twist.angular.z = -this.state.angular_cmd;
          console.log("Rotating at " + twist.angular.z + " rad/s");
          break;
        case "t":
          console.log("Toggling camera tilter");
          var tilter_msg = new roslib.Message({command: 2});
          this.state.tilterCmdPub.publish(tilter_msg);
          break;
        default:
          break;
      }
      console.log(twist)
      this.state.cmdVelPub.publish(twist);
    }
    console.log('key down: ' + event.key);
  }
  handleKeyUp = (event: any) => {
    console.log('key up: ' + event.key);
    if (this.state.motors_enabled) {
      console.log('Stopping motors');
      var twist = new roslib.Message({
        linear : {
          x : 0.0,
          y : 0.0,
          z : 0.0
        },
        angular : {
          x : 0.0,
          y : 0.0,
          z : 0.0
        }
      });
      this.state.cmdVelPub.publish(twist);
    }
  }

  MotorsEnabledSection() {
    if (this.state.motors_enabled) {
      return <h2>Motors enabled</h2>;
    }
    else {
      return <h2>Motors disabled</h2>;
    }
  }
  
  render() {
    const battery = this.state.battery;
    const bumper = this.state.bumper;
    const drive = this.state.drive;
    const fsrs = this.state.fsrs;
    const image = this.state.image;
    const depth = this.state.depth;

    return <div>
      <a href="http://192.168.0.21:8080/admin">Pi-Hole Admin Console</a>
      <div style={{ display: "grid", gridTemplateColumns: "repeat(2, 1fr)", gridGap: 20 }}>
        <div>
          <this.MotorsEnabledSection />
          <Switch onChange={this.handleChangeMotorState} checked={this.state.motors_enabled} />
          <br></br>
          
          <div className="row custom-margin material2">
              <div className="col-xs-6 col-sm-6 col-lg-6 col-md-6"><b>Velocity inputs</b></div>
          </div>
          <div className="row custom-margin custom-padding-5 material2">
              <div className="col-xs-6 col-sm-6 col-lg-6 col-md-6">
                  <TextBoxComponent placeholder="Linear Velocity (m/s)" cssClass="e-outline" floatLabelType="Auto" value={DEFAULT_LINEAR_VEL + ""} onChange={this.handleLinearVelChange}/>
              </div>
              <div className="col-xs-6 col-sm-6 col-lg-6 col-md-6">
                  <TextBoxComponent placeholder="Angular Velocity (rad/s)" cssClass="e-outline" floatLabelType="Auto" value={DEFAULT_ANGULAR_VEL + ""} onChange={this.handleAngularVelChange}/>
              </div>
          </div>
        </div>
        <div>
          <p><b>Controls:</b></p>
          <p>W = drive forward</p>
          <p>A = rotate left</p>
          <p>S = drive backward</p>
          <p>D = rotate right</p>
          <p>T = toggle camera tilter</p>
        </div>
        <hr/>
      </div> 
      <div  style={{ display: "grid", gridTemplateColumns: "repeat(2, 1fr)", gridGap: 20 }}>
        <div hidden={!image}>
          <h2>Color Camera</h2>
          Image format: {image?.format}<br/>
          {/* Data: {image?.data}<br/> */}
          FPS: {image?.fps.toFixed(1)}<br/>
          <img style={{width:"700"}} src={image?.imageSrc}></img>
        </div>
        <div hidden={!depth}>
          <h2>Depth Camera</h2>
          Image format: {image?.format}<br/>
          {/* Data: {image?.data}<br/> */}
          FPS: {depth?.fps.toFixed(1)}<br/>
          <img style={{width:"700"}} src={depth?.imageSrc}></img>
        </div>
      </div>
      <div hidden={!battery}>
        <h2>Battery</h2>
        <p><b>Voltage:</b> {battery?.voltage.toFixed(2)} V</p>
        <p><b>Current:</b> {battery?.current.toFixed(2)} mA</p>
        <hr/>
      </div> 
      <div hidden={!bumper}>
        <h2>Bumper</h2>
        <p><b>Left:</b> {bumper?.left + ""}</p>
        <p><b>Right:</b> {bumper?.right + ""}</p>
        <hr/>
      </div>
      <div hidden={!drive}>
        <h2>Drive</h2>
        <p><b>Left Enc Pos:</b> {drive?.left_enc_pos}</p>
        <p><b>Right Enc Pos:</b> {drive?.right_enc_pos}</p>
        <hr/>
      </div>
      <div hidden={!fsrs}>
        <h2>Force Sensors</h2>
        <p><b>Left:</b> {fsrs?.left}</p>
        <p><b>Right:</b> {fsrs?.right}</p>
      </div>
    </div>
  }
}