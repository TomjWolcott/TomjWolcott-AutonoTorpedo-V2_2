pub use super::*;

pub const UART_TX_QUEUE_DEPTH: usize = 10; // max queued messages

pub static UART_TX_CHANNEL: Channel<
    CriticalSectionRawMutex, 
    Message, 
    UART_TX_QUEUE_DEPTH
> = Channel::new();

#[macro_export]
macro_rules! comms_println {
    ($($arg:tt)*) => {{
        let mut payload: $crate::comms::TextMessage = $crate::comms::TextMessage(heapless::String::new());

        if core::fmt::Write::write_fmt(&mut payload.0, format_args!($($arg)*)).is_err() {
            defmt::warn!("uart_println!: message truncated, formatting overflowed buffer");
        }

        let msg = $crate::comms::Message::from(payload);

        match $crate::comms::tx::UART_TX_CHANNEL.sender().try_send(msg) {
            Ok(_) => {}
            Err(_) => defmt::warn!("uart_println!: tx queue full, dropped"),
        }
    }};
}

/// Call from any task to get a handle for queuing outgoing UART bytes.
pub fn uart_comms_tx_sender()
-> Sender<'static, CriticalSectionRawMutex, Message, UART_TX_QUEUE_DEPTH> {
    UART_TX_CHANNEL.sender()
}

#[embassy_executor::task]
pub async fn uart_comms_tx(mut tx: UartTx<'static, Async>) {
    let receiver = UART_TX_CHANNEL.receiver();

    loop {
        let msg = receiver.receive().await;
        if let Err(e) = tx.write(msg.buffer()).await {
            info!("Uart4_tx error: {:?}", e);
        }
    }
}
