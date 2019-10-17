#include "DMASerial.h"

DMASerial::DMASerial(dma_dev* dev,
                     dma_channel tube,
                     uint8_t buffer_size,
                     dma_irq_handler handler) {
  this->dev = dev;
  this->tube = tube;
  this->irq_handler = handler;
  this->buffer_size = buffer_size;

  initialized = false;
  transfering = false;
  buffer = new uint8_t[buffer_size];

  tube_config.tube_src = buffer;
  tube_config.tube_src_size = DMA_SIZE_8BITS;
  tube_config.tube_dst_size = DMA_SIZE_8BITS;
  tube_config.target_data = 0;
  tube_config.tube_flags = (DMA_FROM_MEM | DMA_MINC_MODE | DMA_TRNS_CMPLT);

  DMAInterrupts::set_interrupt_handler(handler, this);
}

DMASerial::~DMASerial() {
  DMAInterrupts::remove_interrupt_handler(irq_handler);
  delete[] buffer;
}

void DMASerial::init(usart_dev* usart, dma_request_src req_src) {
  usart->regs->CR3 = USART_CR3_DMAT;
  tube_config.tube_req_src = req_src;
  tube_config.tube_dst = &(usart->regs->DR);

  dma_init(dev);
  dma_set_priority(dev, tube, DMA_PRIORITY_LOW);

  initialized = true;
}

int8_t DMASerial::start(void) {
  if (!initialized)
    return 1;
  if (transfering)
    return 2;

  tube_config.tube_nr_xfers = index;

  int8_t error = dma_tube_cfg(dev, tube, &tube_config);
  if (error == DMA_TUBE_CFG_SUCCESS) {
    transfering = true;
    dma_enable(dev, tube);
  }

  return error;
}

bool DMASerial::is_transfering(void) {
  return transfering;
}

void DMASerial::on_completed() {
  transfering = false;
}

void DMASerial::set_data(uint8_t* data, uint8_t size) {
  for (index = 0; index < min(buffer_size, size); index++)
    buffer[index] = data[index];
}

void DMASerial::append_data(uint8_t* data, uint8_t size) {
  uint8_t initial = index;
  while (index - initial < size && index < buffer_size) {
    buffer[index] = data[index - initial];
    index++;
  }
}

void DMASerial::reset_data() {
  index = 0;
}

// DMAInterrupts Implementation

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
