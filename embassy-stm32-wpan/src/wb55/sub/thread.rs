//! Thread transport for the CPU2 Thread stack: the OpenThread CLI channel
//! (IPCC channel 5) and the notification/ack channel (IPCC channel 3).
//!
//! This is enough to drive the OpenThread CLI that lives inside the CPU2
//! Thread firmware. OpenThread API commands (channel 3, CPU1 -> CPU2) are not
//! implemented; notifications on channel 3 are acknowledged and their ID
//! reported.

use core::ptr;

use embassy_stm32::ipcc::{IpccRxChannel, IpccTxChannel};

use crate::cmd::{CmdPacket, CmdSerialStub};
use crate::consts::TlPacketType;
use crate::tables::{
    THREAD_CLI_CMD_BUFFER, THREAD_CLI_NOT_BUFFER, THREAD_NOTIF_RSP_EVT_BUFFER, THREAD_OT_CMD_BUFFER, TL_THREAD_TABLE,
    TL_TRACES_TABLE, TRACES_EVT_QUEUE, ThreadTable, TracesTable,
};
use crate::unsafe_linked_list::LinkedListNode;
use crate::wb55::PacketHeader;

pub struct Thread<'a> {
    ot_cmd_rsp: IpccTxChannel<'a>,
    notification_ack: IpccRxChannel<'a>,
    cli_cmd: IpccTxChannel<'a>,
    cli_notification_ack: IpccRxChannel<'a>,
}

impl<'a> Thread<'a> {
    pub(crate) fn new(
        ot_cmd_rsp: IpccTxChannel<'a>,
        notification_ack: IpccRxChannel<'a>,
        cli_cmd: IpccTxChannel<'a>,
        cli_notification_ack: IpccRxChannel<'a>,
    ) -> Self {
        unsafe {
            // ST calls TL_TRACES_Init() before starting any stack: CPU2 pushes
            // trace events into this queue, and a null queue head is a CPU2
            // crash waiting to happen (THREAD_Init never answers).
            LinkedListNode::init_head(TRACES_EVT_QUEUE.as_mut_ptr() as *mut _);
            TL_TRACES_TABLE.as_mut_ptr().write_volatile(TracesTable {
                traces_queue: TRACES_EVT_QUEUE.as_ptr() as *const _,
            });

            // Same as TL_THREAD_Init() in ST's tl_mbox.c.
            TL_THREAD_TABLE.as_mut_ptr().write_volatile(ThreadTable {
                notack_buffer: THREAD_NOTIF_RSP_EVT_BUFFER.as_mut_ptr().cast(),
                clicmdrsp_buffer: THREAD_CLI_CMD_BUFFER.as_mut_ptr().cast(),
                otcmdrsp_buffer: THREAD_OT_CMD_BUFFER.as_mut_ptr().cast(),
                clinot_buffer: THREAD_CLI_NOT_BUFFER.as_mut_ptr().cast(),
            });
        }

        Self {
            ot_cmd_rsp,
            notification_ack,
            cli_cmd,
            cli_notification_ack,
        }
    }

    pub fn split(self) -> (ThreadOt<'a>, ThreadCliRx<'a>, ThreadNotifRx<'a>) {
        (
            ThreadOt {
                ot_cmd_rsp: self.ot_cmd_rsp,
                _cli_cmd: self.cli_cmd,
            },
            ThreadCliRx {
                cli_notification_ack: self.cli_notification_ack,
            },
            ThreadNotifRx {
                notification_ack: self.notification_ack,
            },
        )
    }
}

/// ST acknowledges both Thread channels by writing the OT ack packet type
/// into the notification buffer before clearing the channel flag.
fn write_ack_type() {
    unsafe {
        let p = (THREAD_NOTIF_RSP_EVT_BUFFER.as_mut_ptr() as *mut u8).add(size_of::<PacketHeader>());
        p.write_volatile(TlPacketType::OtAck as u8);
    }
}

/// OpenThread API calls mirrored to CPU2 over IPCC channel 3, in the format
/// of ST's openthread_api wrappers: a Thread_OT_Cmd_Request_t { ID, Size,
/// Data[Size] } with 32-bit arguments (pointers are addresses in CPU1 RAM,
/// which CPU2 reads and writes directly).
pub struct ThreadOt<'a> {
    ot_cmd_rsp: IpccTxChannel<'a>,
    _cli_cmd: IpccTxChannel<'a>,
}

impl<'a> ThreadOt<'a> {
    /// Issue one API call and return Data[0] of the response (the return
    /// value of the mirrored function).
    pub async fn call(&mut self, id: u32, args: &[u32]) -> u32 {
        const MAX_ARGS: usize = 16;
        let n = args.len().min(MAX_ARGS);
        let mut payload = [0u8; 8 + 4 * MAX_ARGS];
        payload[0..4].copy_from_slice(&id.to_le_bytes());
        payload[4..8].copy_from_slice(&(n as u32).to_le_bytes());
        for (i, a) in args[..n].iter().enumerate() {
            payload[8 + 4 * i..12 + 4 * i].copy_from_slice(&a.to_le_bytes());
        }
        let len = 8 + 4 * n;
        // OpenThread OT command cmdcode range 0x280..0x3DF (ST uses 0x280).
        self.ot_cmd_rsp
            .send(|| unsafe {
                CmdPacket::write_into(THREAD_OT_CMD_BUFFER.as_mut_ptr(), TlPacketType::OtCmd, 0x280, &payload[..len]);
            })
            .await;
        self.ot_cmd_rsp.flush().await;
        unsafe {
            // CPU2 answers in the same buffer, laid out as an event packet:
            // header, type, evtcode, plen, then Thread_OT_Cmd_Request_t.
            let p = (THREAD_OT_CMD_BUFFER.as_ptr() as *const u8).add(size_of::<PacketHeader>() + 3 + 8);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            ptr::read_unaligned(p as *const u32)
        }
    }
}

/// A callback from CPU2 (MsgId_M0toM4_Enum_t) with its first arguments.
#[derive(Clone, Copy, Debug)]
pub struct OtNotification {
    pub id: u32,
    pub size: u32,
    pub data: [u32; 4],
}

pub struct ThreadCliRx<'a> {
    cli_notification_ack: IpccRxChannel<'a>,
}

impl<'a> ThreadCliRx<'a> {
    /// Wait for the next chunk of CLI output from CPU2, copy it into `buf`
    /// and acknowledge it. Returns the number of bytes copied.
    pub async fn receive(&mut self, buf: &mut [u8]) -> usize {
        self.cli_notification_ack
            .receive(|| unsafe {
                let p_serial = (THREAD_CLI_NOT_BUFFER.as_ptr() as *const u8).add(size_of::<PacketHeader>());
                let stub = ptr::read_unaligned(p_serial as *const CmdSerialStub);
                let len = (stub.payload_len as usize).min(buf.len());
                ptr::copy_nonoverlapping(p_serial.add(size_of::<CmdSerialStub>()), buf.as_mut_ptr(), len);
                write_ack_type();
                Some(len)
            })
            .await
    }
}

pub struct ThreadNotifRx<'a> {
    notification_ack: IpccRxChannel<'a>,
}

impl<'a> ThreadNotifRx<'a> {
    /// Wait for the next OpenThread notification from CPU2, copy out its ID
    /// and first arguments, and acknowledge it.
    pub async fn receive(&mut self) -> OtNotification {
        self.notification_ack
            .receive(|| unsafe {
                // EvtPacket: header, then { type, evtcode, plen, payload... }
                let p = (THREAD_NOTIF_RSP_EVT_BUFFER.as_ptr() as *const u8).add(size_of::<PacketHeader>() + 3);
                let rd = |off: usize| ptr::read_unaligned(p.add(off) as *const u32);
                let id = rd(0);
                let size = rd(4);
                let mut data = [0u32; 4];
                for (i, d) in data.iter_mut().enumerate() {
                    if (i as u32) < size {
                        *d = rd(8 + 4 * i);
                    }
                }
                write_ack_type();
                Some(OtNotification { id, size, data })
            })
            .await
    }
}
