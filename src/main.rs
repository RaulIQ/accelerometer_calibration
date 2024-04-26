#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::println;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::I2c;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::hz;
use embassy_time::{Delay, Duration, Timer};
use nalgebra::{Matrix6x3, Matrix6x4, Matrix1x4, Vector3, RowVector4, Matrix4x3};

use gy91::*;
use {defmt_rtt as _, panic_probe as _};

pub const PI: f32 = core::f32::consts::PI;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

struct Positon {
    i: Vector3<f32>,
    j: Vector3<f32>,
    k: Vector3<f32>
}

fn get_motion_vector(g: Vector3<f32>, mpu: &mut Mpu6050<I2c<I2C1>, Delay>) -> Vector3<f32> {
    let mut x = Vector3::<f32>::new(0.0, 0.0, 0.0);
    let nx = x.magnitude();
    let mut number_measurements = 0.0;

    while x.magnitude() == nx {
        for _ in 0..1000 {
            let acc = mpu.get_acc().unwrap();
            if ((acc - g).magnitude()) > 2.0 {
                x += acc;
                number_measurements += 1.0;
            }
        }
    }

    x /= number_measurements;
    x -= g;

    return x
}
fn get_initial_position(g: Vector3<f32>, mpu: &mut Mpu6050<I2c<I2C1>, Delay>) -> Positon {
    println!("Толкните вперед");
    let x = get_motion_vector(g, mpu);

    let y = x.cross(&g);

    let initial_position = Positon {
        i: x,
        j: y,
        k: g
    };

    return initial_position
}

async fn get_raw_measurement_matrix(mpu: &mut Mpu6050<'_, I2c<'_, I2C1>, Delay>) -> Option<Matrix6x4<f32>> {
    let mut raw_measurements = Matrix6x4::zeros();

    println!("z down");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    raw_measurements.set_row(0, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    let initial_position = get_initial_position(acc, mpu);

    println!("z up");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    if (acc.angle(&initial_position.k) * 180.0 / PI) > 150.0 {
        raw_measurements.set_row(1, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    } else {
        println!("ERROR");
        return None;
    };

    println!("y down");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    if (acc.angle(&initial_position.j) * 180.0 / PI) > 150.0 {
        raw_measurements.set_row(2, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    } else {
        println!("ERROR");
        return None;
    };

    println!("y up");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    if (acc.angle(&initial_position.j) * 180.0 / PI) < 30.0 {
        raw_measurements.set_row(3, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    } else {
        println!("ERROR");
        return None;
    };

    println!("x down");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    if (acc.angle(&initial_position.i) * 180.0 / PI) > 150.0 {
        raw_measurements.set_row(4, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    } else {
        println!("ERROR");
        return None;
    };

    println!("x up");
    Timer::after(Duration::from_secs(5)).await;
    let acc = mpu.get_acc().unwrap();
    if (acc.angle(&initial_position.i) * 180.0 / PI) < 30.0 {
        raw_measurements.set_row(5, &RowVector4::new(acc.x, acc.y, acc.z, 1.0));
    } else {
        println!("ERROR");
        return None;
    };

    return Some(raw_measurements)
}

async fn get_calibration_parameters(mpu: &mut Mpu6050<'_, I2c<'_, I2C1>, Delay>) -> Option<Matrix4x3<f32>> {
    let raw_measurements = get_raw_measurement_matrix(mpu).await;
    match raw_measurements {
        Some(T) => {
            let stationary_positions = Matrix6x3::<f32>::new(0.0, 0.0, 1.0,
                                                             0.0, 0.0, -1.0,
                                                             0.0, 1.0, 0.0,
                                                             0.0, -1.0, 0.0,
                                                             1.0, 0.0, 0.0,
                                                             -1.0, 0.0, 0.0);

            let raw_measurement = T;
            let transposed_matrix_raw_measurements = raw_measurement.transpose();
            let inverse_matrix_raw_measurements = (transposed_matrix_raw_measurements * raw_measurement).try_inverse().unwrap();
            let calibration_parameters = inverse_matrix_raw_measurements * transposed_matrix_raw_measurements * stationary_positions;

            return Some(calibration_parameters)
        },
        None => return None
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        NoDma,
        NoDma,
        hz(100_000),
        Default::default()
    );

    let mut delay = Delay;

    let mut mpu = Mpu6050::new(i2c, &mut delay);
    mpu.init().unwrap();

    let calibration_parameters = get_calibration_parameters(&mut mpu).await;
    
    loop {
        
        match calibration_parameters {
            Some(T) => {
                let acc = mpu.get_acc().unwrap();
                let w = Matrix1x4::<f32>::new(acc.x, acc.y, acc.z, 1.0);
                let normalized_data = w * T;
                println!("{} {} {}", normalized_data.x, normalized_data.y, normalized_data.z);

                Timer::after(Duration::from_millis(100)).await;
            },
            None => {
                let acc = mpu.get_acc().unwrap();
                println!("raw measurements {} {} {}", acc.x, acc.y, acc.z);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
        
    }
}