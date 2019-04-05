#include "DMASerial.h"

DMASerial::DMASerial(dma_dev* dev,
                     dma_channel tube,
                     uint8_t size,
                     dma_irq_handler handler) {
  this->dev = dev;
  this->tube = tube;
  this->irq_handler = handler;
  buffer_size = size;

  initialized = false;
  transfering = false;
  buffer = new uint8_t[size];

  tube_config.tube_src = buffer;
  tube_config.tube_src_size = DMA_SIZE_8BITS;
  tube_config.tube_dst_size = DMA_SIZE_8BITS;
  tube_config.target_data = 0;
  tube_config.tube_nr_xfers = size;
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
