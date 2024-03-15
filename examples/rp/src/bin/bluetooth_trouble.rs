//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::future::{pending, Future};
use core::ops::DerefMut;

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::event::{CommandComplete, Event, EventPacketHeader};
use bt_hci::{
    data, param, Controller, ControllerCmdAsync, ControllerCmdSync, ControllerToHostPacket, FromHciBytes, PacketKind,
    ReadHci, WithIndicator, WriteHci,
};
use cyw43_pio::PioSpi;
use defmt::{assert_eq, todo, *};
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join3;
use embassy_futures::yield_now;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use static_cell::StaticCell;
use trouble_host::adapter::{Adapter, HostResources};
use trouble_host::advertise::{AdStructure, AdvertiseConfig, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE};
use trouble_host::attribute::{AttributeTable, Characteristic, CharacteristicProp, Service, Uuid};
use trouble_host::PacketQos;
use {defmt_rtt as _, embassy_time as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");
    let btfw = include_bytes!("../../../../cyw43-firmware/43439A0_btfw.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 224190) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;
    //unwrap!(spawner.spawn(wifi_task(runner)));
    //control.init(clm, false, true).await;

    let controller = MyController {
        runner: Mutex::new(runner),
    };
    static HOST_RESOURCES: StaticCell<HostResources<NoopRawMutex, 4, 32, 27>> = StaticCell::new();
    let host_resources = HOST_RESOURCES.init(HostResources::new(PacketQos::None));

    let adapter: Adapter<'_, NoopRawMutex, _, 2, 4, 1, 1> = Adapter::new(controller, host_resources);
    let config = AdvertiseConfig {
        params: None,
        data: &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x0f, 0x18])]),
            AdStructure::CompleteLocalName("Trouble on Pico W??"),
        ],
    };

    let mut table: AttributeTable<'_, NoopRawMutex, 10> = AttributeTable::new();

    // Generic Access Service (mandatory)
    let mut id = [b'T', b'r', b'o', b'u', b'b', b'l', b'e'];
    let mut appearance = [0x80, 0x07];
    let mut bat_level = [0; 1];
    let handle = {
        let mut svc = table.add_service(Service::new(0x1800));
        let _ = svc.add_characteristic(Characteristic::new(0x2a00, &[CharacteristicProp::Read], &mut id[..]));
        let _ = svc.add_characteristic(Characteristic::new(
            0x2a01,
            &[CharacteristicProp::Read],
            &mut appearance[..],
        ));
        drop(svc);

        // Generic attribute service (mandatory)
        table.add_service(Service::new(0x1801));

        // Battery service
        let mut svc = table.add_service(Service::new(0x180f));

        svc.add_characteristic(Characteristic::new(
            0x2a19,
            &[CharacteristicProp::Read, CharacteristicProp::Notify],
            &mut bat_level,
        ))
    };

    let server = adapter.gatt_server(&table);

    info!("Starting advertising and GATT service");
    let _ = join3(
        adapter.run(),
        async {
            loop {
                match server.next().await {
                    Ok(event) => {
                        info!("Gatt event: {:?}", event);
                    }
                    Err(e) => {
                        error!("Error processing GATT events: {:?}", e);
                    }
                }
            }
        },
        async {
            let conn = adapter.advertise(&config).await.unwrap();
            // Keep connection alive
            let mut tick: u8 = 0;
            loop {
                Timer::after(Duration::from_secs(10)).await;
                tick += 1;
                server.notify(handle, &conn, &[tick]).await.unwrap();
            }
        },
    )
    .await;
}

struct MyController {
    runner: Mutex<NoopRawMutex, cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>>,
}

const BUF_LEN: usize = 512;

impl Controller for MyController {
    type Error = core::convert::Infallible;

    fn write_acl_data(&self, packet: &data::AclPacket) -> impl Future<Output = Result<(), Self::Error>> {
        async {
            let mut runner = self.runner.lock().await;

            let mut buf = [0; BUF_LEN];
            let mut slice = &mut buf[..];
            WithIndicator::new(packet).write_hci_async(&mut slice).await.unwrap();
            let n = BUF_LEN - slice.len();
            let pkt = &buf[..n];
            info!("tx {:02x}", pkt);

            runner.hci_write(pkt).await;
            Ok(())
        }
    }

    fn write_sync_data(&self, packet: &data::SyncPacket) -> impl Future<Output = Result<(), Self::Error>> {
        async {
            let mut runner = self.runner.lock().await;

            let mut buf = [0; BUF_LEN];
            let mut slice = &mut buf[..];
            WithIndicator::new(packet).write_hci_async(&mut slice).await.unwrap();
            let n = BUF_LEN - slice.len();
            let pkt = &buf[..n];
            info!("tx {:02x}", pkt);

            runner.hci_write(pkt).await;
            Ok(())
        }
    }

    fn write_iso_data(&self, packet: &data::IsoPacket) -> impl Future<Output = Result<(), Self::Error>> {
        async {
            let mut runner = self.runner.lock().await;

            let mut buf = [0; BUF_LEN];
            let mut slice = &mut buf[..];
            WithIndicator::new(packet).write_hci_async(&mut slice).await.unwrap();
            let n = BUF_LEN - slice.len();
            let pkt = &buf[..n];
            info!("tx {:02x}", pkt);

            runner.hci_write(pkt).await;
            Ok(())
        }
    }

    fn read<'a>(&self, buf: &'a mut [u8]) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
        pending()
        /*
        async {
            let mut runner = self.runner.lock().await;
            let n = runner.hci_read(buf).await as usize;
            let kind = PacketKind::from_hci_bytes_complete(&buf[3..4]).unwrap();
            let (res, _) = ControllerToHostPacket::from_hci_bytes_with_kind(kind, &buf[4..]).unwrap();
            Ok(res)
        }
         */
    }
}

impl<C> ControllerCmdSync<C> for MyController
where
    C: SyncCmd,
    C::Return: bt_hci::FixedSizeValue,
{
    fn exec(&self, cmd: &C) -> impl Future<Output = Result<C::Return, param::Error>> {
        async {
            let mut runner = self.runner.lock().await;

            let mut buf = [0; BUF_LEN];
            let mut slice = &mut buf[..];
            WithIndicator::new(cmd).write_hci_async(&mut slice).await.unwrap();
            let n = BUF_LEN - slice.len();
            let pkt = &buf[..n];
            info!("tx {:02x}", pkt);

            runner.hci_write(pkt).await;

            //info!("write done, waiting for bt_has_work");
            //while !runner.bt_has_work().await {
            //    yield_now().await;
            //}

            let n = runner.hci_read(&mut buf).await as usize;
            let pkt = &buf[..n];
            info!("rx {:02x}", pkt);

            let (header, remaining) = EventPacketHeader::from_hci_bytes(&pkt[4..]).unwrap();
            assert_eq!(header.code, 0x0e);
            let evt = CommandComplete::from_hci_bytes_complete(remaining).unwrap();
            let res = C::Return::from_hci_bytes_complete(&*evt.return_param_bytes).unwrap();
            Ok(res)
        }
    }
}

impl<C> ControllerCmdAsync<C> for MyController
where
    C: AsyncCmd,
{
    fn exec(&self, cmd: &C) -> impl Future<Output = Result<(), param::Error>> {
        async { todo!() }
    }
}
