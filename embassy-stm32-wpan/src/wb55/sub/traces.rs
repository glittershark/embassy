//! CPU2 trace output (IPCC channel 4, CPU2 -> CPU1). CPU2 queues trace events
//! into the traces queue and raises the channel; each event is returned to
//! the memory manager when its `EvtBox` is dropped, like system events.

use embassy_stm32::ipcc::IpccRxChannel;

use crate::evt::EvtBox;
use crate::sub::mm;
use crate::tables::TRACES_EVT_QUEUE;
use crate::unsafe_linked_list::LinkedListNode;

pub struct Traces<'a> {
    ipcc_traces_channel: IpccRxChannel<'a>,
}

impl<'a> Traces<'a> {
    pub(crate) fn new(ipcc_traces_channel: IpccRxChannel<'a>) -> Self {
        Self { ipcc_traces_channel }
    }

    /// Wait for the next trace event. Its payload is the trace text.
    pub async fn read(&mut self) -> EvtBox<mm::MemoryManager<'_>> {
        self.ipcc_traces_channel
            .receive(|| unsafe {
                if let Some(node_ptr) =
                    critical_section::with(|cs| LinkedListNode::remove_head(cs, TRACES_EVT_QUEUE.as_mut_ptr()))
                {
                    Some(EvtBox::new(node_ptr.cast()))
                } else {
                    None
                }
            })
            .await
    }
}
