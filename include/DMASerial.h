#ifndef DMASERIAL_H_
#define DMASERIAL_H_

#include <Arduino.h>
#include <libmaple/dma.h>
#include <libmaple/usart.h>

/**
 * Enumaration of the DMA IRQ Handlers available.
 */
typedef enum {
  DMA_IRQ_HANDLER_NONE = 0,
  DMA_IRQ_HANDLER_1 = 1,
  DMA_IRQ_HANDLER_2 = 2,
  DMA_IRQ_HANDLER_3 = 3
} dma_irq_handler;

/**
 * Class to encapsulate access of a UART through DMA using stm32 HAL.
 */
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

/**
 * Class to handle DMA IRQs
 */
class DMAInterrupts {
 public:
  static DMASerial* serial1;
  static DMASerial* serial2;
  static DMASerial* serial3;

  static void set_interrupt_handler(dma_irq_handler, DMASerial*);
  static void remove_interrupt_handler(dma_irq_handler);

  static void handler1(void);
  static void handler2(void);
  static void handler3(void);

 private:
  DMAInterrupts();
  ~DMAInterrupts();
};

#endif
