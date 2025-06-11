use futures_util::stream::SplitSink;
use futures_util::SinkExt;
use log::{info, warn};
use socketcan::{tokio::CanFdSocket, CanAnyFrame, EmbeddedFrame, Frame, Id};
use socketcan::{CanDataFrame, CanFrame, StandardId};

// todo monitor heartbeat

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NMTState {
    // Copied from CiA301, 7.3.2.2 NMT states
    // Warning: They have different values when used in NMT and HEARTBEAT
    Initialising,
    ResetApplication,
    ResetCommunication,
    PreOperational,
    Operational,
    Stopped,
}

// Function to set the NMT state of a node
pub async fn set_nmt_state(
    node_id: u8,
    target_state: NMTState,
    tx: &mut SplitSink<CanFdSocket, CanAnyFrame>,
) -> Result<(), anyhow::Error> {
    if (node_id > 127) {
        return Err(anyhow::anyhow!("Invalid node ID"));
    }
    let can_id = Id::Standard(StandardId::new(0).unwrap());
    let state_code = match target_state {
        NMTState::Operational => 0x01,
        NMTState::Stopped => 0x02,
        NMTState::PreOperational => 0x80,
        NMTState::ResetApplication => 0x81,
        NMTState::ResetCommunication => 0x82,
        _ => return Err(anyhow::anyhow!("Invalid target NMT state")),
    };
    let f = CanFrame::Data(CanDataFrame::new(can_id, &[state_code, node_id]).unwrap());
    tx.send(f.into()).await?;
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum MotorControlMode {
    // The motor is locked and cannot move
    Lock = 0,
    Position(i32) = 1,
    Speed(f32) = 2,
    Torque(f32) = 3,
}

impl TryFrom<u8> for MotorControlMode {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(MotorControlMode::Lock),
            1 => Ok(MotorControlMode::Position(0)),
            2 => Ok(MotorControlMode::Speed(0.0)),
            3 => Ok(MotorControlMode::Torque(0.0)),
            _ => Err(()),
        }
    }
}

impl Into<u8> for MotorControlMode {
    fn into(self) -> u8 {
        match self {
            MotorControlMode::Lock => 0,
            MotorControlMode::Position(_) => 1,
            MotorControlMode::Speed(_) => 2,
            MotorControlMode::Torque(_) => 3,
        }
    }
}

#[derive(Debug, Clone)]
pub struct XstdPcw {
    canopen_id: u8,
    read_control_modes: [MotorControlMode; 2],
    target_control_modes: [MotorControlMode; 2],
    encoder_resolutions: [u32; 2],
    speed_mapping_maxs: [f32; 2],
    speed_mapping_mins: [f32; 2],
    torque_mapping_maxs: [f32; 2],
    torque_mapping_mins: [f32; 2],
    motor_temp_zero: [i16; 2],
    driver_temp_zero: [i16; 2],
}

impl XstdPcw {
    pub fn get_canopen_id(&self) -> u8 {
        self.canopen_id
    }
    fn uint_to_float(x: u32, x_min: f32, x_max: f32, bits: u32) -> f32 {
        // Converts an unsigned int to a float, given range and number of bits
        let span = x_max - x_min;
        let offset = x_min;
        let scale = ((1 << bits) - 1) as f32;
        x as f32 * span / scale + offset
    }
    fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u32) -> u32 {
        // Converts a float to an unsigned int, given range and number of bits
        let span = x_max - x_min;
        let offset = x_min;
        let scale = ((1 << bits) - 1) as f32;
        ((x - offset) * scale / span) as u32
    }
    pub fn new(canopen_id: u8) -> Self {
        Self {
            canopen_id,
            read_control_modes: [MotorControlMode::Lock; 2],
            target_control_modes: [MotorControlMode::Lock; 2],
            encoder_resolutions: [0; 2],
            speed_mapping_maxs: [0.0; 2],
            speed_mapping_mins: [0.0; 2],
            torque_mapping_maxs: [0.0; 2],
            torque_mapping_mins: [0.0; 2],
            motor_temp_zero: [0; 2],
            driver_temp_zero: [0; 2],
        }
    }

    pub fn set_control_mode(&mut self, mode: [MotorControlMode; 2]) {
        self.target_control_modes = mode;
    }

    // Assumes can id is correct
    pub async fn process_can_msg(&mut self, msg: CanAnyFrame) -> Result<(), ()> {
        if msg.data().len() != 24 {
            warn!(
                "Received message with unexpected length: {:?}",
                msg.data().len()
            );
            return Err(());
        }

        let m0_speed_mapping_max = self.speed_mapping_maxs[0];
        let m0_speed_mapping_min = self.speed_mapping_mins[0];
        let m0_torque_mapping_max = self.torque_mapping_maxs[0];
        let m0_torque_mapping_min = self.torque_mapping_mins[0];
        let m1_speed_mapping_max = self.speed_mapping_maxs[1];
        let m1_speed_mapping_min = self.speed_mapping_mins[1];
        let m1_torque_mapping_max = self.torque_mapping_maxs[1];
        let m1_torque_mapping_min = self.torque_mapping_mins[1];
        let m0_encoder_resolution = self.encoder_resolutions[0];
        let m1_encoder_resolution = self.encoder_resolutions[1];

        let m0_pos =
            i32::from_le_bytes([msg.data()[0], msg.data()[1], msg.data()[2], msg.data()[3]]);
        let m1_pos =
            i32::from_le_bytes([msg.data()[4], msg.data()[5], msg.data()[6], msg.data()[7]]);
        let m0_speed = u16::from_le_bytes([msg.data()[8], msg.data()[9]]);
        let m1_speed = u16::from_le_bytes([msg.data()[10], msg.data()[11]]);
        let m0_speed = Self::uint_to_float(
            m0_speed as u32,
            m0_speed_mapping_min,
            m0_speed_mapping_max,
            16,
        );
        let m1_speed = Self::uint_to_float(
            m1_speed as u32,
            m1_speed_mapping_min,
            m1_speed_mapping_max,
            16,
        );
        let m0_torque = u16::from_le_bytes([msg.data()[12], msg.data()[13]]);
        let m1_torque = u16::from_le_bytes([msg.data()[14], msg.data()[15]]);
        let m0_torque = Self::uint_to_float(
            m0_torque as u32,
            m0_torque_mapping_min,
            m0_torque_mapping_max,
            16,
        );
        let m1_torque = Self::uint_to_float(
            m1_torque as u32,
            m1_torque_mapping_min,
            m1_torque_mapping_max,
            16,
        );
        let m0_read_mode = MotorControlMode::try_from(msg.data()[16])?;
        let m1_read_mode = MotorControlMode::try_from(msg.data()[17])?;
        self.read_control_modes[0] = m0_read_mode;
        self.read_control_modes[1] = m1_read_mode;
        // Underlaying protocol only report mode, will not send data via pdo. So just display a string here.
        let m0_read_mode_str = match m0_read_mode {
            MotorControlMode::Lock => "Lock",
            MotorControlMode::Position(_) => "Position",
            MotorControlMode::Speed(_) => "Speed",
            MotorControlMode::Torque(_) => "Torque",
        };
        let m1_read_mode_str = match m1_read_mode {
            MotorControlMode::Lock => "Lock",
            MotorControlMode::Position(_) => "Position",
            MotorControlMode::Speed(_) => "Speed",
            MotorControlMode::Torque(_) => "Torque",
        };
        let m0_error_code = msg.data()[18];
        let m1_error_code = msg.data()[19];
        let m0_mt_temp = msg.data()[20] as i16 + self.motor_temp_zero[0];
        let m1_mt_temp = msg.data()[21] as i16 + self.motor_temp_zero[1];
        let m0_driver_temp = msg.data()[22] as i16 + self.driver_temp_zero[0];
        let m1_driver_temp = msg.data()[23] as i16 + self.driver_temp_zero[1];

        // TODO: For our user, you send the decoded data else where
        // As a demo here, we just print them out
        info!(
            "Motor1: Mode {}, Position {}, Speed {}, Torque {}, Error Code {}, Motor Temp {}, Driver Temp {}",
            m0_read_mode_str, m0_pos, m0_speed, m0_torque, m0_error_code, m0_mt_temp, m0_driver_temp
        );
        info!(
            "Motor2: Mode {}, Position {}, Speed {}, Torque {}, Error Code {}, Motor Temp {}, Driver Temp {}",
            m1_read_mode_str, m1_pos, m1_speed, m1_torque, m1_error_code, m1_mt_temp, m1_driver_temp
        );

        Ok(())
    }

    pub async fn initialize(&mut self, tx: &mut SplitSink<CanFdSocket, CanAnyFrame>) {
        // todo Read OD to get mapping values
        self.encoder_resolutions = [0xFFFF; 2];
        self.speed_mapping_maxs = [50.0; 2];
        self.speed_mapping_mins = [-50.0; 2];
        self.torque_mapping_maxs = [10.0; 2];
        self.torque_mapping_mins = [-10.0; 2];
        // todo verify other od settings are correct

        set_nmt_state(self.canopen_id, NMTState::Operational, tx)
            .await
            .unwrap();
    }

    pub async fn generate_control_frame(pcws: &[XstdPcw]) -> CanAnyFrame {
        let mut data: Vec<u8> = vec![];
        for pcw in pcws {
            data.push(pcw.target_control_modes[0].into());
            data.push(pcw.target_control_modes[1].into());
        }
        for pcw in pcws {
            for (i, mode) in pcw.target_control_modes.iter().enumerate() {
                match mode {
                    MotorControlMode::Lock => {}
                    MotorControlMode::Position(p) => {
                        let d = p.to_le_bytes();
                        data.extend_from_slice(&d);
                    }
                    MotorControlMode::Speed(s) => {
                        let s = Self::float_to_uint(
                            *s,
                            pcw.speed_mapping_mins[i],
                            pcw.speed_mapping_maxs[i],
                            16,
                        ) as u16;
                        let d = s.to_le_bytes();
                        data.extend_from_slice(&d);
                    }
                    MotorControlMode::Torque(t) => {
                        let t = Self::float_to_uint(
                            *t,
                            pcw.torque_mapping_mins[i],
                            pcw.torque_mapping_maxs[i],
                            16,
                        ) as u16;
                        let d = t.to_le_bytes();
                        data.extend_from_slice(&d);
                    }
                }
            }
        }
        let id = socketcan::Id::Standard(socketcan::StandardId::new(0x181).unwrap());
        if data.len() <= 8 {
            let frame = socketcan::CanDataFrame::new(id, &data).unwrap();
            socketcan::CanAnyFrame::Normal(frame)
        } else {
            // let fd_frame =
            //     socketcan::CanFdFrame::with_flags(id, &data, socketcan::id::FdFlags::BRS).unwrap();
            let fd_frame = socketcan::CanFdFrame::new(id, &data).unwrap();
            socketcan::CanAnyFrame::Fd(fd_frame)
        }
    }
}
