use embassy_sync::pubsub::PubSubChannel;

pub use super::*;

pub const WEB_RX_QUEUE_DEPTH: usize = 20; // max queued messages
pub const WEB_RX_SUBS_MAX: usize = 20; // max queued messages

pub static WEBSITE_RX_PUB_SUB: PubSubChannel<
    CriticalSectionRawMutex, 
    Message, 
    WEB_RX_QUEUE_DEPTH, 
    WEB_RX_SUBS_MAX, 
    2 // UART & BLUETOOTH
> = PubSubChannel::new();

pub const UART_RX_MSG_SIZE: usize = 1000; // max bytes per message, tune as needed

#[embassy_executor::task]
pub async fn uart_comms_rx(mut rx: UartRx<'static, Async>) {
    let mut buffer = [0u8; UART_RX_MSG_SIZE];
    let mut bytes_left = 0;
    let mut byte_offset = 0;
    let publisher = WEBSITE_RX_PUB_SUB.publisher().unwrap();

    // comms_println!("COMMY RX");
    loop {
        if bytes_left == 0 {
            byte_offset = 0;
            bytes_left = match rx.read_until_idle(&mut buffer).await {
                Ok(num_bytes) => num_bytes,
                Err(e) => {
                    info!("Uart4_rx error1: {:?}", e);
                    continue;
                }
            };
            // comms_println!("ACTUAL # RX'd: {bytes_left}");
        } else {
            match Message::try_new(&buffer[byte_offset..]) {
                Ok(msg) => {
                    bytes_left -= msg.length();
                    byte_offset += msg.length();
                    // comms_println!("Got MSG: {}", msg);
                    publisher.publish(msg).await;
                } 
                Err(e) => {
                    bytes_left = 0;
                    comms_println!("UART4_RX Error: {e:?}");
                }
            }
        }
    }
}