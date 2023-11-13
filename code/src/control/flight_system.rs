use crate::control::radio::{Radio, RadioCommand};
use crate::math::functions::*;
use crate::sensors::imu::ICM_20948;
use crate::sensors::imu::{Accelerometer, Gyroscope, Sensor};
use core::u16;
use defmt::info;
use defmt::Format;
use embedded_hal::PwmPin;
use hal::pwm::{Channel, FreeRunning, Pwm0, Pwm1, Pwm2, Pwm3, A};
use hal::sio::Spinlock0 as CoreStateLock;
use hal::uart::{ReadError, ReadErrorType};
use libm::{cosf, powf, roundf, acosf};

use hal::multicore::{Core, Stack};
use rp2040_hal as hal;
use rp2040_hal::rom_data::float_funcs::fsqrt;

use super::radio::RadioError;

#[derive(Clone, Copy, Debug, Format)]
pub struct DroneCoreState {
    true_acceleration: [f32; 3], //best estimate
    desired_acceleration: [f32; 3],
    true_angle: [f32; 2], //best estimate
    desired_angle: [f32; 2],
    angular_velocity: [f32; 3],
    desired_twist: f32,
    aux: f32,
    current_command: DroneCommand,
    raw_command: RadioCommand, // vestigial
}

#[derive(Clone, Copy)]
pub struct DronePeripheralState {} // for when I add more things

#[derive(Clone, Copy, Format, Debug, PartialEq)]
pub enum DroneCommand {
    Land,
    Calibrate,
    FullManual,
    NormalControl,
    FallOutOfTheSky, // probably better than going up
}

static mut CORESTATE: Option<DroneCoreState> = None;

static mut CORE1_STACK: Stack<8192> = Stack::new();

// arbitrary max values
const MAX_TWIST: f32 = 100.0; //dps
const MAX_TILT: f32 = 20.0; //degrees
const CAL_LENGTH: u32 = 1000; //number of cycles
const IMU_FAILURE_THRESHOLD: u8 = 100;
const RADIO_TEMPORARY_FAILURE_THRESHOLD: u16 = 100;
const RADIO_FULL_FAILURE_THRESHOLD: u16 = 2000;
// 1-2ms PWM
const MAX_THROTTLE: u16 = 0xCCCC;
const MIN_THROTTLE: u16 = 0x6666;
const TICKS2SEC: f32 = 1.0f32 / 1000000.0f32;

const MAX_THRUST: f32 = 20.0; // m/s^2

//percent difference for manual control
const MAX_THROTTLE_DIFFERENCE: f32 = 0.1;

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,
    last_time: u64,
    timer: hal::Timer,
}

impl FlightSystem {
    pub fn new(
        fl: Channel<Pwm0, FreeRunning, A>,
        bl: Channel<Pwm1, FreeRunning, A>,
        br: Channel<Pwm2, FreeRunning, A>,
        fr: Channel<Pwm3, FreeRunning, A>,
        timer: hal::timer::Timer,
    ) -> Self {
        Self {
            fl,
            bl,
            br,
            fr,
            last_time: timer.get_counter().ticks(),
            timer,
        }
    }

    pub fn start(
        &mut self,
        mut delay: cortex_m::delay::Delay,
        mut imu: ICM_20948,
        radio: Radio,
        core1: &mut Core,
    ) -> ! {
        if !cfg!(debug_assertions) {
            delay.delay_us(4206969);
        } // delay before calibration if im not debuggin g

        imu.init(&mut delay).unwrap();
        imu.update_all(&mut delay).unwrap();
        let initial_command = radio.get_command();
        unsafe {
            let _clock = CoreStateLock::claim();
            CORESTATE = Some(
                DroneCoreState {
                    true_acceleration: imu.get_acc(),
                    desired_acceleration: [0.0,0.0,0.0],
                    true_angle: cartesian_to_polar_theta(imu.get_acc(), true),
                    desired_angle: [0.0,0.0],
                    angular_velocity: [0.0,0.0,0.0],
                    desired_twist: 0.0,
                    aux: initial_command.aux,
                    current_command: DroneCommand::FallOutOfTheSky,
                    raw_command: initial_command,
                }
            )
        }
        let mut g = 0.0;
        for _ in 0..CAL_LENGTH {
            imu.update_raw_acc().unwrap();
            g += cartesian_to_polar_magnitude(imu.get_acc());
        }
        g /= CAL_LENGTH as f32;
        let _core1task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            Self::core1_task(g, delay, imu, radio)
        });
        self.core0_task();
    }

    fn core1_task(
        //read from stuff and update states
        g: f32,
        mut delay: cortex_m::delay::Delay,
        mut imu: ICM_20948,
        mut radio: Radio,
    ) -> ! {
        let mut failed_imu: u8 = 0;
        let mut failed_radio: u16 = 0;
        let mut newimu: bool = false;
        let mut newradio: bool = false;
        loop {
            // theres a race condition somewhere here i think
            newimu = match imu.update_all(&mut delay) {
                Ok(()) => {
                    failed_imu = 0;
                    true
                }
                Err(e) => {
                    failed_imu += 1;
                    e.info();
                    false
                }
            };
            newradio = match radio.read() {
                Ok(()) => {
                    failed_radio = 0;
                    // info!("works {}", num_overrun);
                    true
                }
                Err(e) => {
                    failed_radio += 1;
                    match e {
                        RadioError::NoNewData => (),
                        RadioError::ChecksumError => info!("radio no work :("),
                        RadioError::ReadError(r) => match r {
                            Some(t) => match t {
                                rp2040_hal::uart::ReadErrorType::Overrun => {
                                    info!("overrun")
                                }
                                _ => info!("not"),
                            },
                            None => info!("first bad"),
                        },
                    };
                    false
                }
            };
            unsafe {
                let mut newstate: DroneCoreState = CORESTATE.clone().unwrap();
                if newimu {
                    let measured_acceleration: [f32; 3] = imu.get_acc();
                    let measured_angular_velocity: [f32; 3] = imu.get_gyr();
                    if measured_acceleration[0] == 0.0 && measured_acceleration[1] == 0.0 {
                        newstate.true_angle = [0.0,0.0];
                    } else {
                        let projection_constant: f32 = 1.0f32 / fsqrt(measured_acceleration[0] * measured_acceleration[0] + measured_acceleration[1] * measured_acceleration[1]);
                        newstate.true_angle = [acosf(measured_acceleration[0] * projection_constant), acosf(measured_acceleration[1] * projection_constant)];
                    }
                    // TODO
                    // newstate.true_acceleration = [ , , -g];

                } else if failed_imu > IMU_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::FallOutOfTheSky;
                }
                if newradio {
                    let radio_command: RadioCommand = radio.get_command();
                    // TODO
                    // figure out how to denormalize (?)
                    newstate.desired_acceleration = [radio_command.x_throttle, radio_command.y_throttle, radio_command.z_throttle + g];
                    if newstate.desired_acceleration[0] == 0.0 && newstate.desired_acceleration[1] == 0.0 {
                        newstate.desired_angle = [0.0,0.0];
                    } else {
                        // TODO
                        // let projection_constant: f32 = 1.0f32 / fsqrt(newstate.desired_acceleration[0] * newstate.desired_acceleration[0] + newstate.desired_acceleration[1] * newstate.desired_acceleration[1]);
                        // newstate.desired_angle = [acosf(newstate.desired_acceleration[0] * projection_constant), acosf(newstate.desired_acceleration[1] * projection_constant)];

                    }
                    newstate.current_command = match radio_command.mode_select {
                        1 => DroneCommand::NormalControl,
                        2 => DroneCommand::FullManual,
                        _ => DroneCommand::FallOutOfTheSky,
                    };
                    newstate.raw_command = radio_command;
                } else if failed_radio > RADIO_TEMPORARY_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::Land;
                } else if failed_radio > RADIO_FULL_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::FallOutOfTheSky;
                }
                let _clock = CoreStateLock::claim();
                CORESTATE = Some(newstate);
            }
        }
    }

    fn core0_task(&mut self) -> ! { // must do dt here
        // let mut maxovern: MaxOverN<500> = MaxOverN::new();
        loop {
            let mut current_state = None;
            unsafe {
                let _clock = CoreStateLock::claim();
                current_state = CORESTATE.clone();
            }
            let current_state = current_state.unwrap();

            let dticks: u64 = self.timer.get_counter().ticks() - self.last_time;
            self.last_time += dticks;
            let dt: f32 = dticks as f32 * TICKS2SEC;

            info!("{} Hz", 1.0 / dt);

            match current_state.current_command {
                DroneCommand::FullManual => self.full_manual(current_state.raw_command),
                DroneCommand::NormalControl => {
                    self.normal_control(current_state, dt)
                }
                DroneCommand::Calibrate => self.set_speeds([current_state.raw_command.z_throttle; 4]),
                _ => self.fall_out_of_the_sky(),
            }
        }
    }

    fn normal_control(
        &mut self,
        current_state: DroneCoreState,
        dt: f32,
    ) {
        // black magic
    }

    fn unreliable_speeds(&mut self, new_speeds: &mut [f32; 4]) {
        let mut max = new_speeds[0];
        for i in 1..4 {
            if new_speeds[i] > max {
                max = new_speeds[i];
            }
        }
        if max < 0.05 {
            self.set_speeds([0.0; 4]);
        } else {
            if max > 1.0 {
                for i in 0..4 {
                    new_speeds[i] /= max;
                }
            }
            self.set_speeds(*new_speeds);
        }
    }
    fn set_speeds(&mut self, speeds: [f32; 4]) {
        // clockwise starting at Front Left
        let mut speedsu16: [u16; 4] = [0; 4]; //inefficient!!
        for i in 0..4 {
            speedsu16[i] = (speeds[i] * (MAX_THROTTLE - MIN_THROTTLE) as f32) as u16 + MIN_THROTTLE;
        }
        self.fl.set_duty(speedsu16[0]);
        self.fr.set_duty(speedsu16[1]);
        self.br.set_duty(speedsu16[2]);
        self.bl.set_duty(speedsu16[3]);
    }

    fn full_manual(&mut self, command: RadioCommand) {
        let mut speeds: [f32; 4] = [command.z_throttle; 4];
        speeds[0] += MAX_THROTTLE_DIFFERENCE
            * (command.x_throttle - command.y_throttle + command.twist_throttle);
        speeds[1] += MAX_THROTTLE_DIFFERENCE
            * (-command.x_throttle - command.y_throttle - command.twist_throttle);
        speeds[2] += MAX_THROTTLE_DIFFERENCE
            * (-command.x_throttle + command.y_throttle + command.twist_throttle);
        speeds[3] += MAX_THROTTLE_DIFFERENCE
            * (command.x_throttle + command.y_throttle - command.twist_throttle);
        let mut max = speeds[0];
        for i in 1..4 {
            if speeds[i] > max {
                max = speeds[i];
            }
        }
        if max == 0.0 {
            self.set_speeds([0.0; 4]);
        } else if max > 1.0 {
            for i in 0..4 {
                speeds[i] /= max;
            }
            self.set_speeds(speeds);
        } else {
            self.set_speeds(speeds);
        }
    }

    fn fall_out_of_the_sky(&mut self) {
        self.set_speeds([0.0; 4]);
    }
}
