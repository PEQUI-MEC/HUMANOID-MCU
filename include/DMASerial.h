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
            uint8_t size,
            dma_irq_handler handler);
  ~DMASerial(void);

  void init(usart_dev* usart = USART1,
            dma_request_src req_src = DMA_REQ_SRC_USART1_TX);
  int8_t start(void);
  bool is_transfering(void);
  void on_completed(void);

  // TODO: Refazer a função de alterar os dados
  void set_data(String);

  dma_dev* dev;
  dma_channel tube;

 private:
  dma_tube_config tube_config;
  dma_irq_handler irq_handler;

  bool initialized;
  uint8_t size;
  uint8_t* data;
  bool transfering;
};

#endif
