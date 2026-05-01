#include "../Inc/UDPCLIENTRAW.h"

typedef struct
{
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t port;
} ip_address_t ;

ip_address_t stm32_addr = { .a = 192, .b = 168, .c = 1, .d = 123, .port = 1555 };
ip_address_t pc_addr    = { .a = 192, .b = 168, .c = 1, .d = 124, .port = 1556 };

struct udp_pcb *upcb;
ip_addr_t stm32IPaddr;    //-- stm32
ip_addr_t pcIPaddr;  	  //-- NUC/MINI PC

udpTx_t udp_tx;
volatile udpRx_t udp_rx;

char udp_rx_buffer[UDP_BUFFER_SIZE_RX];
char udp_tx_buffer[UDP_BUFFER_SIZE_TX] = "ABC";
uint32_t rx_packet_count = 0;
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
// ==============================
// INIT UDP
// ==============================
void udpClient_connect(void)
{
    err_t err;

    upcb = udp_new();

    IP_ADDR4(&stm32IPaddr,
             stm32_addr.a,
             stm32_addr.b,
             stm32_addr.c,
             stm32_addr.d);

    udp_bind(upcb, &stm32IPaddr, stm32_addr.port);

    IP_ADDR4(&pcIPaddr,
             pc_addr.a,
             pc_addr.b,
             pc_addr.c,
             pc_addr.d);

    err = udp_connect(upcb, &pcIPaddr, pc_addr.port);

    if (err == ERR_OK)
    {
        udp_recv(upcb, udp_receive_callback, NULL);
    }
}

void udpClient_send(void)
{
  struct pbuf *txBuf;

  // 1. Hitung panjang data asli (Header 3 byte + isi struct)
  int data_len = 3 + sizeof(udpTx_t);

  // 2. Pastikan data_len tidak melebihi ukuran buffer fisik
  if (data_len > UDP_BUFFER_SIZE_TX) data_len = UDP_BUFFER_SIZE_TX;

  // 3. Salin struct ke buffer setelah header "ABC"
  memcpy(udp_tx_buffer + 3, &udp_tx, sizeof(udpTx_t));

  // 4. Alokasikan pbuf sesuai panjang data asli, bukan ukuran maksimal buffer
  txBuf = pbuf_alloc(PBUF_TRANSPORT, data_len, PBUF_POOL);

  if (txBuf != NULL)
  {
    pbuf_take(txBuf, udp_tx_buffer, data_len);
    udp_sendto(upcb, txBuf, &pcIPaddr, pc_addr.port);
    pbuf_free(txBuf);
  }
}

void udp_receive_callback(void *arg,struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	rx_packet_count++;
    if (p == NULL) return;

    if (p->len < (3 + sizeof(udpRx_t))) {
        pbuf_free(p);
        return;
    }

    memcpy(udp_rx_buffer, p->payload, p->len);

    if (udp_rx_buffer[0] != 'A' || udp_rx_buffer[1] != 'B' || udp_rx_buffer[2] != 'C') {
        pbuf_free(p);
        return;
    }

    memcpy(&udp_rx, udp_rx_buffer + 3, sizeof(udpRx_t));

    pbuf_free(p);
}


