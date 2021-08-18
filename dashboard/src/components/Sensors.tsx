export interface DodobotBatteryState {
  voltage: number,
  current: number
}

export interface DodobotBumperState {
  left: boolean,
  right: boolean
}

export interface DodobotDriveState {
  left_setpoint: number,
  right_setpoint: number,
  left_enc_pos: number,
  right_enc_pos: number,
  left_enc_speed: number,
  right_enc_speed: number,
  left_bumper: boolean,
  right_bumper: boolean
}

export interface DodobotFSRsState {
  left: number,
  right: number
}

export interface DodobotGripperState {
  position: number,
  force_threshold: number
}

export interface DodobotLinearState {
  position: number,
  has_error: boolean,
  is_homed: boolean,
  is_active: boolean,
  command_type: number,
  command_value: number,
  max_speed: number,
  acceleration: number
}

export interface CompressedImage {
  format?: string,
  data?: number[],
  imageSrc: string,
  fps: number
}