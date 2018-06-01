#define DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termio.h>
#include <time.h>
#include <termios.h>
#include <linux/serial.h>
#include <pthread.h>
#include <err.h>
#include <errno.h>
#include <list>
#include <iostream>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include <VescUart.h>
#include <datatypes.h>

static bool uart_running = true;

static void signalHandler(int signal) {
        printf("\033[?25h");
        switch(signal) {
        case SIGABRT:
        case SIGTERM:
        case SIGINT:
        case SIGKILL:
                break;
        default:
                std::cout << "Caught unsupported signal:" << std::endl;
                return;
        }

        std::cout << "Caught signal. Terminating.." << std::endl;

        uart_running = false;
}

static void *uart_main_trampoline(void *arg)
{
  unsigned long count = 0;
  struct bldcMeasure measuredVal;
        while(uart_running) {
          //int len=0;
          //len = ReceiveUartMessage(message);
          //if (len > 0)
          //{
          //	len = PackSendPayload(message, len);
          //	len = 0;
          //}

          VescUartSetRPM(3000);
          // VescUartSetServoPosition(0.3);

          // if (VescUartGetValue(measuredVal)) {
          //   printf("Loop: %d\n", count++);
          //   SerialPrint(measuredVal);
          // } else {
          //   printf("Failed to get data!\n");
          // }

        }
        return 0;
}

static void print_usage() {
  printf("Usage: VescUartSample -d <device>\n");
}

int main(int argc, char** argv){
  int ret;
  char *file = NULL;
  int option = 0;
  int fd;
  int baud = 115200;
  int databits = 8;
  int stopbits = 1;
  int parity = 0;
  pthread_t uart_task_id;

  /* initialize random seed: */
  srand (time(NULL));

  while ((option = getopt(argc, argv,"d:")) != -1) {
    switch (option) {
      case 'd' :
        file = optarg;
        break;
      default:
        print_usage();
        exit(EXIT_FAILURE);
    }
  }

  if (optind == 1) {
    print_usage();
    exit(EXIT_FAILURE);
  }

  if(file == NULL) {
    exit(EXIT_FAILURE);
  }

  // Register signals for cleanup.
  signal(SIGABRT, signalHandler);
	signal(SIGTERM, signalHandler);
	signal(SIGINT, signalHandler);
	signal(SIGKILL, signalHandler);

  std::string filename = file;
  // fd = open(filename.c_str(), O_RDWR, 0);
  // if (fd < 0) {
  //   fprintf(stderr, "open <%s> error %s\n", filename.c_str(), strerror(errno));
  //   return -1;
  // }

  // if (uart_setup_port(fd, baud, databits, parity, stopbits)) {
  //   fprintf(stderr, "setup_port error %s\n", strerror(errno));
  //   close(fd);
  //   return -2;
  // }

  SerialPortInit(filename.c_str());

  uart_running = true;

	ret = pthread_create(&uart_task_id, NULL, uart_main_trampoline, NULL);
	if (ret < 0) {
		warnx("task start failed: %d", errno);
    exit(EXIT_FAILURE);
	}

  pthread_join(uart_task_id, NULL);
  close(fd);

  return 0;
}
