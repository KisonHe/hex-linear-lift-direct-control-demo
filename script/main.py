import asyncio
import logging
import can

from xstd_pcw import XstdPcw, MotorControlMode, MitData
from xstd_pcw import set_nmt_state, NMTState

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

async def main():
    # Create CAN bus
    bus = can.Bus(channel='can0', bustype='socketcan', fd=True)
    
    # Create PCW instance
    pcw = XstdPcw(0x10)
    
    # Initialize PCW
    # Send NMT command to enter operational mode
    set_nmt_state(pcw.canopen_id, NMTState.OPERATIONAL, bus)
    await asyncio.sleep(0.1)
    
    # Set control mode (modify as needed)
    pcw.set_control_mode([
        MotorControlMode.speed(1.0),
        MotorControlMode.speed(1.0)
    ])
    # pcw.set_control_mode([
    #     MotorControlMode.torque(1.0),
    #     MotorControlMode.torque(1.0)
    # ])
    # pcw.set_control_mode([
    #     MotorControlMode.mit(MitData(0.0, 1.0, 0.0, 1.0, 0.0)),
    #     MotorControlMode.mit(MitData(0.0, 1.0, 0.0, 1.0, 0.0))
    # ])
    
    # Message processing task
    async def process_messages():
        while True:
            msg = bus.recv(timeout=0.1)
            if msg:
                pcw.process_can_msg(msg)
            await asyncio.sleep(0.001)

    # Control command sending task
    async def send_control_commands():
        while True:
            msg = XstdPcw.generate_control_frame([pcw])
            bus.send(msg)
            await asyncio.sleep(0.02)  # 50Hz

    # Start tasks
    await asyncio.gather(
        process_messages(),
        send_control_commands()
    )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        logger.error(f"Error: {e}") 