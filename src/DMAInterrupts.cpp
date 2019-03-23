#include <DMAInterrupts.h>

DMASerial* DMAInterrupts::serial1 = 0;
DMASerial* DMAInterrupts::serial2 = 0;
DMASerial* DMAInterrupts::serial3 = 0;

void DMAInterrupts::set_interrupt_handler(dma_irq_handler handler,
                                          DMASerial* serial) {
  switch (handler) {
    case DMA_IRQ_HANDLER_NONE:
      break;

    case DMA_IRQ_HANDLER_1:
      serial1 = serial;
      dma_attach_interrupt(serial->dev, serial->tube, DMAInterrupts::handler1);
      break;

    case DMA_IRQ_HANDLER_2:
      serial2 = serial;
      dma_attach_interrupt(serial->dev, serial->tube, DMAInterrupts::handler2);
      break;

    case DMA_IRQ_HANDLER_3:
      serial3 = serial;
      dma_attach_interrupt(serial->dev, serial->tube, DMAInterrupts::handler3);
      break;
  }
}

void DMAInterrupts::remove_interrupt_handler(dma_irq_handler handler) {
  switch (handler) {
    case DMA_IRQ_HANDLER_NONE:
      break;

    case DMA_IRQ_HANDLER_1:
      dma_detach_interrupt(serial1->dev, serial1->tube);
      serial1 = 0;
      break;

    case DMA_IRQ_HANDLER_2:
      dma_detach_interrupt(serial2->dev, serial2->tube);
      serial2 = 0;
      break;

    case DMA_IRQ_HANDLER_3:
      dma_detach_interrupt(serial3->dev, serial3->tube);
      serial3 = 0;
      break;
  }
}

void DMAInterrupts::handler1() {
  if (serial1 != 0)
    serial1->on_completed();
}

void DMAInterrupts::handler2() {
  if (serial2 != 0)
    serial2->on_completed();
}

void DMAInterrupts::handler3() {
  if (serial3 != 0)
    serial3->on_completed();
}
