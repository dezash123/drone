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
use libm::{cosf, powf, roundf};

use hal::multicore::{Core, Stack};
use rp2040_hal as hal;

use super::radio::RadioError;

#[derive(Clone, Copy, Debug, Format)]
pub struct DroneCoreState {
    true_acceleration: [f32; 3], //best estimate
    desired_acceleration: [f32; 3],
    true_angle: [f32; 2], //best estimate
    desired_angle: [f32; 2],
    measurement_time: u64,
    current_command: DroneCommand,
}

#[derive(Clone, Copy)]
pub struct DronePeripheralState {}

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
const CAL_LENGTH: u32 = 1000; //number of cycles
const IMU_FAILURE_THRESHOLD: u8 = 100;
const RADIO_TEMPORARY_FAILURE_THRESHOLD: u16 = 100;
const RADIO_FULL_FAILURE_THRESHOLD: u16 = 2000;
// 1-2ms PWM
const MAX_THROTTLE: u16 = 0xCCCC;
const MIN_THROTTLE: u16 = 0x6666;

const MAX_THRUST: f32 = 20.0; // m/s^2

//percent difference for manual control
const MAX_THROTTLE_DIFFERENCE: f32 = 0.1;

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,
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
        } // delay before calibration if im not debuggin, g

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
                    measurement_time: self.timer.get_counter().ticks(),
                    current_command: DroneCommand::FallOutOfTheSky,
                }
            )
        }
        let mut g = 0.0;
        for _ in 0..CAL_LENGTH {
            imu.update_raw_acc().unwrap();
            g += cartesian_to_polar_magnitude(imu.get_acc());
        }
        g /= CAL_LENGTH as f32;
        let _core1task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, || {
            Self::core1_task(delay, imu, radio)
        });
        self.core0_task();
    }

    fn core1_task(
        //read from stuff and update states
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
            if newimu {
                let measured_acceleration = imu.get_acc();
                let measured_angular_velocity = imu.get_gyr();
                // black magic

            }
            unsafe {
                let mut newstate = CORESTATE.clone().unwrap();
                if newimu {
                    newstate.tr = imu.get_acc();
                    newstate.last_gyr = imu.get_gyr();
                } else if failed_imu > IMU_FAILURE_THRESHOLD {
                    newstate.current_command = DroneCommand::FallOutOfTheSky;
                }
                if newradio {
                    newstate.last_input = radio.get_command();
                    newstate.current_command = match newstate.last_input.mode_select {
                        1 => DroneCommand::NormalControl,
                        2 => DroneCommand::FullManual,
                        _ => DroneCommand::FallOutOfTheSky,
                    }
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

    fn core0_task(&mut self) -> ! {
        let ticks2sec = 1.0 / 1000000.0;
        let mut maxovern: MaxOverN<500> = MaxOverN::new();
        loop {
            let mut current_state = None;
            unsafe {
                let _clock = CoreStateLock::claim();
                current_state = CORESTATE.clone();
            }
            let current_state = current_state.unwrap();

            let aux = current_state.last_input.aux;
            let throttle = current_state.last_input.z_throttle;

            let a = current_state.last_acc;
            let w = current_state.last_gyr;

            let dticks = self.timer.get_counter().ticks() - self.last_time;
            self.last_time += dticks;
            let dt = dticks as f32 * ticks2sec;

            self.gyro_filter.push(a, w, dt);
            let theta = self.gyro_filter.last_value;

            let desired_angle: [f32; 2] = [
                current_state.last_input.x_throttle * MAX_TILT,
                current_state.last_input.y_throttle * MAX_TILT,
            ];
            let desired_wz: f32 = current_state.last_input.twist_throttle * MAX_TWIST;
            // let maxar = maxovern.get(a_r);
            // info!("{}g", roundf(maxar * invg * 10000.0) / 10000.0);
            info!("{} Hz", 1.0 / dt);
            // info!("{}", aux);
            // info!("{}", throttle);
            match current_state.current_command {
                DroneCommand::FullManual => self.full_manual(current_state.last_input),
                DroneCommand::NormalControl => {
                    self.normal_control(throttle, desired_angle, desired_wz, theta, w[2], dt, aux)
                }
                DroneCommand::Calibrate => self.set_speeds([throttle; 4]),
                _ => self.fall_out_of_the_sky(),
            }
        }
    }

    fn normal_control(
        &mut self,
        throttle: f32,
        desired_angle: [f32; 2],
        desired_wz: f32,
        theta: [f32; 2],
        wz: f32,
        dt: f32,
        aux: f32,
    ) {

        self.unreliable_speeds(&mut new_speeds);

        info!("{}", tuner);
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
