#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use core::{mem, ptr};

use esp_hal::{
    gpio::{Input, Output, Pull},
    spi::{Spi, SpiMode},
    peripherals,
    prelude::*,
    Cpu,
};
use esp_backtrace as _;

// FreeRTOS / ESP bindings (FFI)
use esp_idf_sys::{
    xTaskCreatePinnedToCore, xQueueCreate, xQueueSend, xQueueReceive, xQueueSendFromISR,
    xTaskGetCurrentTaskHandle, xTaskNotifyFromISR, vTaskDelay, pdMS_TO_TICKS, portMAX_DELAY,
    esp_timer_get_time, QueueHandle_t, TaskHandle_t, pdMS_TO_TICKS as pd_ms_to_ticks,
};

use esp_wifi::wifi::esp_now::{EspNow, PeerInfo};

// Конфигурация
const ESPNOW_CHANNEL: u8 = 1;
const CPI_DEFAULT: u16 = 800;
const QUEUE_DEPTH: usize = 2;

// Регистры сенсора
const REG_MOTION: u8 = 0x02;
const REG_DELTA_X_L: u8 = 0x03;
const REG_DELTA_Y_L: u8 = 0x05;
const REG_POWER_UP_RESET: u8 = 0x3A;
const REG_SET_RESOLUTION: u8 = 0x47;
const REG_RES_X_LOW: u8 = 0x48;
const REG_RES_X_HIGH: u8 = 0x49;
const REG_RES_Y_LOW: u8 = 0x4A;
const REG_RES_Y_HIGH: u8 = 0x4B;

// Структура пакета
#[repr(C, packed)]
#[derive(Copy, Clone)]
struct MousePacket {
    dx: i8,
    dy: i8,
    buttons: u8,
    timestamp: u32,
}

// Глобальные переменные (атомарные для безопасности)
static MOTION_FLAG: AtomicBool = AtomicBool::new(false);
static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
static mut LAST_BUTTON_STATE: u8 = 0;
static mut PACKET_COUNT: u32 = 0;
static mut DROP_COUNT: u32 = 0;

// FreeRTOS queue handle (shared between tasks / ISR)
static mut MOUSE_QUEUE_HANDLE: QueueHandle_t = ptr::null_mut();

// ============ ДРАЙВЕР СЕНСОРА ============
fn adns_com_begin(cs: &mut Output) {
    cs.set_low();
    // small busy wait - FreeRTOS tick granularity is ms; use esp_timer_get_time based busy-wait
    // keep original timings: 1 us
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 1 {}
}

fn adns_com_end(cs: &mut Output) {
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 1 {}
    cs.set_high();
}

fn adns_write_reg(spi: &mut Spi<peripherals::SPI2>, cs: &mut Output, reg: u8, val: u8) {
    adns_com_begin(cs);
    let tx_buf = [reg | 0x80, val];
    let _ = spi.write(&tx_buf);
    adns_com_end(cs);
    // 20 us wait
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 20 {}
}

fn adns_read_reg(spi: &mut Spi<peripherals::SPI2>, cs: &mut Output, reg: u8) -> u8 {
    adns_com_begin(cs);
    let mut rx_buf = [0u8];
    let tx_buf = [reg & 0x7F];
    let _ = spi.write(&tx_buf);
    // 35 us
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 35 {}
    let _ = spi.transfer(&mut rx_buf, &[0]);
    adns_com_end(cs);
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 1 {}
    rx_buf[0]
}

fn twos_comp(b: u8) -> i8 {
    if b & 0x80 != 0 {
        -((!b + 1) & 0xFF) as i8
    } else {
        b as i8
    }
}

fn adns_init(spi: &mut Spi<peripherals::SPI2>, cs: &mut Output, reset: &mut Output) {
    // Аппаратный сброс
    reset.set_low();
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 100 {}
    reset.set_high();
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 10_000 {} // 10 ms

    // Программный сброс
    adns_write_reg(spi, cs, REG_POWER_UP_RESET, 0x5A);
    let t0 = unsafe { esp_timer_get_time() };
    while unsafe { esp_timer_get_time() } - t0 < 5_000 {} // 5 ms

    // Чтение для очистки буферов
    let _ = adns_read_reg(spi, cs, REG_MOTION);
    let _ = adns_read_reg(spi, cs, REG_DELTA_X_L);
    let _ = adns_read_reg(spi, cs, REG_DELTA_Y_L);
}

fn adns_set_cpi(spi: &mut Spi<peripherals::SPI2>, cs: &mut Output, cpi: u16) {
    let mut val = cpi;
    if val < 50 { val = 50; }
    if val > 26000 { val = 26000; }

    val = val / 50;
    adns_write_reg(spi, cs, REG_RES_X_LOW, (val & 0xFF) as u8);
    adns_write_reg(spi, cs, REG_RES_X_HIGH, ((val >> 8) & 0xFF) as u8);
    adns_write_reg(spi, cs, REG_RES_Y_LOW, (val & 0xFF) as u8);
    adns_write_reg(spi, cs, REG_RES_Y_HIGH, ((val >> 8) & 0xFF) as u8);
    adns_write_reg(spi, cs, REG_SET_RESOLUTION, 0x01);
}

fn read_motion(spi: &mut Spi<peripherals::SPI2>, cs: &mut Output) -> (i8, i8) {
    let motion = adns_read_reg(spi, cs, REG_MOTION);
    if motion & 0x80 != 0 {
        let dx_raw = adns_read_reg(spi, cs, REG_DELTA_X_L);
        let dy_raw = adns_read_reg(spi, cs, REG_DELTA_Y_L);
        (twos_comp(dx_raw), twos_comp(dy_raw))
    } else {
        (0, 0)
    }
}

// ============ КНОПКИ ============
fn read_buttons(btn1: &Input, btn2: &Input) -> u8 {
    let mut state = 0;
    if btn1.is_low() { state |= 0x01; }
    if btn2.is_low() { state |= 0x02; }
    state
}

// ============ ОБРАБОТКА ПРЕРЫВАНИЯ ============
static mut MOT_PIN: MaybeUninit<Input> = MaybeUninit::uninit();

#[esp_hal::interrupt]
fn GPIO() {
    if let Some(mot) = unsafe { MOT_PIN.as_mut() } {
        if mot.is_low() {
            // set flag for producer
            MOTION_FLAG.store(true, Ordering::SeqCst);

            // Optionally notify a task from ISR (if you want immediate wake)
            // unsafe {
            //     let mut higher_priority_task_woken: i32 = 0;
            //     xTaskNotifyFromISR(task_handle, 0x01, 0, &mut higher_priority_task_woken);
            // }
        }
    }
}

// ============ ЗАДАЧА ESP-NOW (ядро 1) ============
extern "C" fn espnow_task(_: *mut core::ffi::c_void) {
    // Blocking task context. Initialize WiFi and ESP-NOW here (same as before but non-async).
    esp_println::println!("ESP-NOW task starting on core 1");

    // Note: esp_wifi's esp-now API used previously was async. For a FreeRTOS task
    // you must use a synchronous/blocking API or run a local executor.
    // The snippet below follows the same init path but assumes `espnow.send_blocking`
    // exists; replace with the crate-specific synchronous send, or wrap the async
    // send calls in a small executor.
    let p = unsafe { peripherals::Peripherals::steal() };
    let (wifi, _) = esp_wifi::wifi::new(p.MOD, p.RADIO_CLK).unwrap();
    let mut espnow = EspNow::new(&wifi).unwrap();

    let broadcast_addr = [0xFF; 6];
    espnow.add_peer(PeerInfo {
        peer_addr: broadcast_addr,
        channel: ESPNOW_CHANNEL,
        encrypt: false,
        ..Default::default()
    }).unwrap();

    loop {
        // Receive packet from FreeRTOS queue (blocking)
        let mut packet = MousePacket { dx:0, dy:0, buttons:0, timestamp:0 };
        let received = unsafe {
            xQueueReceive(
                MOUSE_QUEUE_HANDLE,
                &mut packet as *mut _ as *mut core::ffi::c_void,
                portMAX_DELAY,
            )
        };
        if received != 0 {
            // serialize bytes and send
            let bytes = unsafe {
                core::slice::from_raw_parts(
                    &packet as *const _ as *const u8,
                    mem::size_of::<MousePacket>(),
                )
            };

            // Replace this with a blocking send API or wrap the async send in a local executor.
            // Here we attempt the same API but not awaiting; adapt to the espnow crate you use.
            match espnow.send(&broadcast_addr, bytes) {
                Ok(_) => unsafe { PACKET_COUNT += 1; },
                Err(_) => unsafe { DROP_COUNT += 1; },
            }
        }
    }
}

// ============ ОСНОВНАЯ ФУНКЦИЯ (запуск FreeRTOS задач) ============
#[no_mangle]
pub extern "C" fn main() -> ! {
    let p = peripherals::Peripherals::take();
    esp_println::println!("Wireless Mouse starting...");

    // Настройка пинов сенсора
    let mut cs = Output::new(p.GPIO5, Level::High);
    let mut reset = Output::new(p.GPIO6, Level::High);
    let mut mot = Input::new(p.GPIO1, Pull::Up);

    // Сохраняем MOT для прерывания
    unsafe { MOT_PIN.write(mot); }

    // Настройка SPI
    let mut spi = Spi::new(
        p.SPI2,
        p.GPIO2,  // SCK
        p.GPIO3,  // MOSI
        p.GPIO4,  // MISO
        10_000_000u32,
        SpiMode::Mode3,
    );

    // Инициализация сенсора
    adns_init(&mut spi, &mut cs, &mut reset);
    adns_set_cpi(&mut spi, &mut cs, CPI_DEFAULT);

    // Настройка кнопок
    let btn1 = Input::new(p.GPIO12, Pull::Up);
    let btn2 = Input::new(p.GPIO11, Pull::Up);

    // Настройка прерывания
    unsafe {
        esp_hal::interrupt::enable(
            esp_hal::interrupt::Interrupt::GPIO,
            esp_hal::interrupt::Priority::Priority1
        ).ok();

        // enable falling edge on MOT pin (listener kept as before)
        let mot_ref = MOT_PIN.as_mut().unwrap();
        mot_ref.listen(esp_hal::gpio::Event::FallingEdge);
    }

    // Create FreeRTOS queue
    unsafe {
        MOUSE_QUEUE_HANDLE = xQueueCreate(QUEUE_DEPTH as u32, mem::size_of::<MousePacket>() as u32);
        if MOUSE_QUEUE_HANDLE.is_null() {
            esp_println::println!("Failed to create queue");
            loop { unsafe { vTaskDelay(pd_ms_to_ticks(1000)); } }
        }
    }

    // Create ESP-NOW task pinned to core 1
    unsafe {
        let mut task_handle: TaskHandle_t = ptr::null_mut();
        let ret = xTaskCreatePinnedToCore(
            Some(espnow_task),
            b"espnow\0".as_ptr() as *const i8,
            8192, // stack (adjust as needed)
            ptr::null_mut(),
            5, // priority
            &mut task_handle,
            1, // core 1
        );
        if ret == 0 {
            esp_println::println!("Failed to create espnow task");
            loop { unsafe { vTaskDelay(pd_ms_to_ticks(1000)); } }
        }
    }

    esp_println::println!("Wireless Mouse ready");

    // Основной цикл (producer) on core 0
    let mut last_stats = unsafe { esp_timer_get_time() / 1000 }; // ms

    loop {
        // Читаем кнопки в каждом цикле
        let buttons = read_buttons(&btn1, &btn2);
        BUTTON_STATE.store(buttons, Ordering::SeqCst);

        // Проверяем флаг движения
        if MOTION_FLAG.load(Ordering::SeqCst) {
            MOTION_FLAG.store(false, Ordering::SeqCst);

            // Читаем движение
            let (dx, dy) = read_motion(&mut spi, &mut cs);
            let last_buttons = unsafe { LAST_BUTTON_STATE };

            // Отправляем если есть движение или изменились кнопки
            if dx != 0 || dy != 0 || buttons != last_buttons {
                let packet = MousePacket {
                    dx,
                    dy,
                    buttons,
                    timestamp: (unsafe { esp_timer_get_time() } as u32),
                };

                // Отправляем в очередь (неблокирующе)
                let sent = unsafe {
                    xQueueSend(
                        MOUSE_QUEUE_HANDLE,
                        &packet as *const _ as *const core::ffi::c_void,
                        0,
                    )
                };
                if sent == 0 {
                    unsafe { DROP_COUNT += 1; }
                } else {
                    unsafe { LAST_BUTTON_STATE = buttons; }
                }
            }
        }
        // Если нет движения, но изменились кнопки
        else if buttons != unsafe { LAST_BUTTON_STATE } {
            let packet = MousePacket {
                dx: 0,
                dy: 0,
                buttons,
                timestamp: (unsafe { esp_timer_get_time() } as u32),
            };

            let sent = unsafe {
                xQueueSend(
                    MOUSE_QUEUE_HANDLE,
                    &packet as *const _ as *const core::ffi::c_void,
                    0,
                )
            };
            if sent == 0 {
                unsafe { DROP_COUNT += 1; }
            } else {
                unsafe { LAST_BUTTON_STATE = buttons; }
            }
        }

        // Статистика каждые 10 секунд
        let now_ms = unsafe { esp_timer_get_time() / 1000 };
        if now_ms - last_stats > 10_000 {
            last_stats = now_ms;
            unsafe {
                esp_println::println!(
                    "Sent: {}, Drops: {}, Free: {}, Buttons: 0x{:02X}",
                    PACKET_COUNT,
                    DROP_COUNT,
                    QUEUE_DEPTH as u32, // queue length API not used here; implement if needed
                    LAST_BUTTON_STATE
                );
            }
        }

        // Preserve original ~550 µs delay (busy-wait). This keeps sensor timing identical.
        let t0 = unsafe { esp_timer_get_time() };
        while unsafe { esp_timer_get_time() } - t0 < 550 {}
    }
}