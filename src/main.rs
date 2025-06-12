use crate::xstd_pcw::MitData;
use crate::xstd_pcw::MotorControlMode;
use crate::xstd_pcw::XstdPcw;
use clap::Parser;
use futures_util::SinkExt;
use futures_util::StreamExt;
use log::error;
use log::info;
use socketcan::tokio::CanFdSocket;
use socketcan::CanFilter;
use socketcan::SocketOptions;
use std::time::Duration;

mod xstd_pcw;

#[derive(Parser, Debug, Clone)]
#[command(version, about, long_about = None)]
struct Args {
    /// Can interface name, e.g. can0
    #[arg(long, short)]
    can_interface: String,
}

#[tokio::main]
async fn main() {
    env_logger::init_from_env(
        env_logger::Env::default().filter_or(env_logger::DEFAULT_FILTER_ENV, "info"),
    );

    let args = Args::parse();

    // Check if the interface exists
    let interfaces = socketcan::available_interfaces().expect("Error getting can interfaces");
    if !interfaces.contains(&args.can_interface) {
        error!("Required interface {} not found", &args.can_interface);
        std::process::exit(1);
    }
    // Check if interface is up. Bring up can bus might require root privileges, which is not ideal. So we just panic if it's down.
    {
        let ifi = nix::net::if_::if_nametoindex(args.can_interface.as_str())
            .expect("Error opening can bus");
        let interface = socketcan::CanInterface::open_iface(ifi);
        let d = interface
            .details()
            .expect("Failed to get interface details");
        if !d.is_up {
            error!("Interface {} is down", args.can_interface);
            std::process::exit(1);
        }
    }

    // CAN bus should be ready by now
    let pcws = [
        // In this demo, we will only control one pcw
        std::sync::Arc::new(tokio::sync::Mutex::new(XstdPcw::new(0x10))),
        // std::sync::Arc::new(tokio::sync::Mutex::new(XstdPcw::new(0x11))),
        // std::sync::Arc::new(tokio::sync::Mutex::new(XstdPcw::new(0x12))),
        // std::sync::Arc::new(tokio::sync::Mutex::new(XstdPcw::new(0x13))),
    ];
    let canbus = args.can_interface.clone();
    // Spawn message process thread
    for pcw in pcws.clone() {
        // Initialize PCW
        {
            let canbus = CanFdSocket::open(canbus.as_str()).unwrap();
            let (mut tx, _) = canbus.split();
            pcw.lock().await.initialize(&mut tx).await;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        let pcw_arc = pcw.clone();
        let (_, mut rx) = {
            let canbus = CanFdSocket::open(canbus.as_str()).unwrap();
            let filter = CanFilter::new(pcw.lock().await.get_canopen_id() as u16 as u32, 0x7F);
            canbus.set_filters(&[filter]).unwrap();
            canbus.split()
        };
        tokio::spawn(async move {
            loop {
                let f = rx.next().await.unwrap().unwrap();
                {
                    if let Err(e) = pcw_arc.lock().await.process_can_msg(f) {
                        error!("Error processing CAN message: {:?}", e);
                    }
                }
            }
        });
    }

    // PCW Control Command Receive Thread
    let pcw_arcs = pcws.clone();
    tokio::spawn(async move {
        // TODO as user, change these targets to control the PCW
        // For example, use gilrs with a gamepad, or use ros to recieve and calculate speed, etc.
        // Here for the sake of simplicity, we just set the targets to Speed 1.0
        // We provide a few examples here, uncomment the ones you want to use
        let mt1_target = MotorControlMode::Speed(1.0);
        let mt2_target = MotorControlMode::Speed(1.0);
        // let mt1_target = MotorControlMode::Mit(MitData::torque(1.0));
        // let mt2_target = MotorControlMode::Mit(MitData::torque(1.0));
        // let mt1_target = MotorControlMode::Mit(MitData::new(0.0, 1.0, 0.0, 1.0, 0.0));
        // let mt2_target = MotorControlMode::Mit(MitData::new(0.0, 1.0, 0.0, 1.0, 0.0));
        loop {
            tokio::time::sleep(Duration::from_millis(10)).await;
            pcw_arcs[0]
                .lock()
                .await
                .set_control_mode([mt1_target, mt2_target]);
        }
    });

    // PCW Control Command Send Thread
    // Set to your desired control frequency. Here we use 50Hz.
    let mut tick = tokio::time::interval(Duration::from_millis(20));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let (mut tx, _) = {
        let canbus = CanFdSocket::open(canbus.as_str()).unwrap();
        let filter = CanFilter::new(0 as u32, 0x1FF);
        canbus.set_filters(&[filter]).unwrap();
        canbus.split()
    };
    loop {
        tick.tick().await;
        {
            let f = XstdPcw::generate_control_frame(&[
                pcws[0].lock().await.clone(),
                // pcws[1].lock().await.clone(),
                // pcws[2].lock().await.clone(),
                // pcws[3].lock().await.clone(),
            ])
            .await;
            tx.send(f).await.unwrap();
        }
    }
}
