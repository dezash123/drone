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
    last_input: RadioCommand,
    last_acc: [f32; 3], // x, y, z
    last_gyr: [f32; 3],
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
const MAX_TILT: f32 = 20.0; //degrees
const CAL_LENGTH: u32 = 1000; //number of cycles
const IMU_FAILURE_THRESHOLD: u8 = 100;
const RADIO_TEMPORARY_FAILURE_THRESHOLD: u16 = 100;
const RADIO_FULL_FAILURE_THRESHOLD: u16 = 2000;
// 1-2ms PWM
const MAX_THROTTLE: u16 = 0xCCCC;
const MIN_THROTTLE: u16 = 0x6666;
// "multishot" (30kHz pwm) couldn't get it to work
// const MAX_THROTTLE: u16 = 50000;
// const MIN_THROTTLE: u16 = 10000;

// PIDS!!!!!
const KPX: f32 = 0.0;
const KIX: f32 = 0.0;
const KDX: f32 = 0.0;
const KPY: f32 = 0.0;
const KIY: f32 = 0.0;
const KDY: f32 = 0.0;
const KPZ: f32 = 0.06;
const KIZ: f32 = 0.0;
const KDZ: f32 = 0.0;

// weight on accelerometer measurements
const A_WEIGHT: f32 = 0.3;

// instantaneous angle measurement error at which switch to accelerometer trig
const MAX_THETA_ERROR: f32 = 2.0;

const MAX_XY_A: f32 = 1.07;

//percent difference for manual control
const MAX_THROTTLE_DIFFERENCE: f32 = 0.1;

pub struct FlightSystem {
    fl: Channel<Pwm0, FreeRunning, A>,
    bl: Channel<Pwm1, FreeRunning, A>,
    br: Channel<Pwm2, FreeRunning, A>,
    fr: Channel<Pwm3, FreeRunning, A>,
    xpd: PD,
    ypd: PD,
    zpd: PD,
    gyro_filter: GyroFilter,
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
            xpd: PD::new(KPX, KDX),
            ypd: PD::new(KPY, KDY),
            zpd: PD::new(KPZ, KDZ),
            gyro_filter: GyroFilter::new(MAX_XY_A, MAX_THETA_ERROR, A_WEIGHT),
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
        } // delay for calibration if im not debuggin

        imu.init(&mut delay).unwrap();

        let initial_command = radio.get_command();
        unsafe {
            let _clock = CoreStateLock::claim();
            CORESTATE = Some(DroneCoreState {
                last_input: initial_command,
                last_acc: imu.get_acc(),
                last_gyr: imu.get_gyr(),
                current_command: DroneCommand::FallOutOfTheSky,
            })
        }
        let mut g = 0.0;
        for _ in 0..CAL_LENGTH {
            imu.update_raw_acc().unwrap();
            g += cartesian_to_polar_magnitude(imu.get_acc());
        }
        g /= CAL_LENGTH as f32;
        self.gyro_filter.g_measured2g_true = 1.0 / g;
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
        let mut newimu = false;
        let mut newradio = false;
        let mut num_overrun = 0;
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
                                    num_overrun += 1;
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
                let mut newstate = CORESTATE.clone().unwrap();
                if newimu {
                    newstate.last_acc = imu.get_acc();
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

            // info!(
            //     "{:#?}d",
            //     [
            //         roundf(theta[0] * 1000.0) / 1000.0,
            //         roundf(theta[1] * 1000.0) / 1000.0
            //     ]
            // );
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
        // start .0001 go factors of 10 for kd
        // start 1 for kp factors of 10
        // this was bad advice ^ lol
        let tuner = aux;

        let mut new_speeds = [throttle; 4];

        let error_x = desired_angle[0] - theta[0];
        let error_y = desired_angle[1] - theta[1];
        let error_wz = desired_wz + wz; // it was the wrong way so add??

        self.zpd.k_p = tuner * 100.0;

        let x = if error_x > 1.0 || error_x < -1.0 {
            self.xpd.get_next(error_x, dt)
        } else {
            0.0
        };
        let y = if error_y > 1.0 || error_y < -1.0 {
            self.ypd.get_next(error_y, dt)
        } else {
            0.0
        };
        let z = if error_wz > 0.0 || error_wz < 1.0 {
            self.zpd.get_next(error_wz, dt)
        } else {
            0.0
        };

        new_speeds[0] += x - y + z;
        new_speeds[1] += -x - y - z;
        new_speeds[2] += -x + y + z;
        new_speeds[3] += x + y - z;

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
