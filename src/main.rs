use clap::Parser;
use clap::Subcommand;
use futures_util::SinkExt;
use futures_util::StreamExt;
use log::debug;
use log::error;
use log::info;
use log::trace;
use log::warn;
use prost::Message;
use serde::Deserialize;
use serde::Serialize;
use std::time::Duration;
use tokio_tungstenite::tungstenite::{
    protocol::{frame::coding::CloseCode, CloseFrame},
    Message as WsMessage,
};
use tokio_tungstenite::MaybeTlsStream;

pub mod base_backend {
    include!(concat!(env!("OUT_DIR"), "/_.rs"));
}

#[derive(Debug, Parser)]
struct Cli {
    #[arg(help = "WebSocket URL to connect to (e.g. ws://localhost:8080)")]
    url: String,
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
#[clap(disable_help_subcommand = true)]
enum Commands {
    #[command(about = "Print info and do nothing")]
    Print,
    #[command(about = "Ask the lift to calibrate")]
    Calibrate,
    #[command(about = "Brake the lift")]
    Brake,
    #[command(about = "Move the lift to a position")]
    Move {
        #[arg(help = "Position to move to (in pulses)")]
        position: i64,
    },
    #[command(about = "Set the speed of the lift")]
    SetSpeed {
        #[arg(help = "Speed to set (in pulses per second)")]
        speed: u32,
    },
}

#[tokio::main]
async fn main() {
    env_logger::init_from_env(
        env_logger::Env::default().filter_or(env_logger::DEFAULT_FILTER_ENV, "info"),
    );

    let args = Cli::parse();

    ctrlc::set_handler(move || {
        std::process::exit(0);
    })
    .expect("Error setting Ctrl-C handler");

    let url = args.url;
    let (mut ws_stream, _) = tokio_tungstenite::connect_async(&url).await.unwrap();
    match ws_stream.get_ref() {
        MaybeTlsStream::Plain(stream) => {
            stream.set_nodelay(true).unwrap();
        }
        _ => warn!("set_nodelay not implemented for TLS streams"),
    }
    match args.command {
        Commands::Print => {
            while let Some(msg) = ws_stream.next().await {
                let msg = msg.unwrap();
                match msg {
                    WsMessage::Text(txt) => {
                        info!("Text message: {:?}", txt);
                    }
                    WsMessage::Binary(bin) => {
                        let msg = base_backend::ApiUp::decode(bin.to_vec().as_slice()).unwrap();
                        if let Some(status) = msg.status {
                            match status {
                                base_backend::api_up::Status::LinearLiftStatus(lift_status) => {
                                    info!("Received lift status: {:?}", lift_status);
                                }
                                _ => {}
                            }
                        }
                    }
                    WsMessage::Close(frame) => {
                        debug!("Remote close message: {:?}", frame);
                        break;
                    }
                    _ => {}
                }
            }
        }
        // Send calibrate command and quit
        Commands::Calibrate => {
            let linear_lift_command = base_backend::LinearLiftCommand {
                command: Some(base_backend::linear_lift_command::Command::Calibrate(true)),
            };
            let cmd = base_backend::ApiDown {
                down: Some(base_backend::api_down::Down::LinearLiftCommand(
                    linear_lift_command,
                )),
            };
            ws_stream
                .send(WsMessage::Binary(cmd.encode_to_vec().into()))
                .await
                .unwrap();
            std::process::exit(0);
        }
        Commands::Brake => {
            let linear_lift_command = base_backend::LinearLiftCommand {
                command: Some(base_backend::linear_lift_command::Command::Brake(true)),
            };
            let cmd = base_backend::ApiDown {
                down: Some(base_backend::api_down::Down::LinearLiftCommand(
                    linear_lift_command,
                )),
            };
            ws_stream
                .send(WsMessage::Binary(cmd.encode_to_vec().into()))
                .await
                .unwrap();
            std::process::exit(0);
        }
        Commands::Move { position } => {
            let linear_lift_command = base_backend::LinearLiftCommand {
                command: Some(base_backend::linear_lift_command::Command::TargetPos(
                    position,
                )),
            };
            let cmd = base_backend::ApiDown {
                down: Some(base_backend::api_down::Down::LinearLiftCommand(
                    linear_lift_command,
                )),
            };
            ws_stream
                .send(WsMessage::Binary(cmd.encode_to_vec().into()))
                .await
                .unwrap();
            std::process::exit(0);
        }
        Commands::SetSpeed { speed } => {
            let linear_lift_command = base_backend::LinearLiftCommand {
                command: Some(base_backend::linear_lift_command::Command::SetSpeed(speed)),
            };
            let cmd = base_backend::ApiDown {
                down: Some(base_backend::api_down::Down::LinearLiftCommand(
                    linear_lift_command,
                )),
            };
            ws_stream
                .send(WsMessage::Binary(cmd.encode_to_vec().into()))
                .await
                .unwrap();
            std::process::exit(0);
        }
    }
    // Avoid exiting before ws buffer is flushed
    tokio::time::sleep(Duration::from_secs(1)).await;
}
