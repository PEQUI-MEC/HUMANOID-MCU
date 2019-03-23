#ifndef DMAINTERRUPTS_H_
#define DMAINTERRUPTS_H_

#include <libmaple/dma.h>
#include <libmaple/usart.h>

class DMASerial;

typedef enum {
  DMA_IRQ_HANDLER_NONE = 0,
  DMA_IRQ_HANDLER_1 = 1,
  DMA_IRQ_HANDLER_2 = 2,
  DMA_IRQ_HANDLER_3 = 3
} dma_irq_handler;

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

#include <DMASerial.h>

#endif
