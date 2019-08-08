#ifndef DMASERIAL_H_
#define DMASERIAL_H_

#include <Arduino.h>
#include <DMAInterrupts.h>
#include <libmaple/dma.h>
#include <libmaple/usart.h>

class DMASerial {
 public:
  DMASerial(dma_dev* dev,
            dma_channel tube,
            uint8_t buffer_size,
            dma_irq_handler handler);
  ~DMASerial(void);

  void init(usart_dev* usart = USART2,
            dma_request_src req_src = DMA_REQ_SRC_USART2_TX);
  int8_t start(void);
  bool is_transfering(void);
  void on_completed(void);

  void set_data(uint8_t* data, uint8_t size);
  void append_data(uint8_t* data, uint8_t size);
  void reset_data();

  dma_dev* dev;
  dma_channel tube;

 private:
  dma_tube_config tube_config;
  dma_irq_handler irq_handler;

  bool initialized;
  uint8_t buffer_size;
  uint8_t* buffer;
  uint8_t index;
  bool transfering;
};

#endif
